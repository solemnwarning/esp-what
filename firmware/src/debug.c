/* ESP-WHAT - ESP Wireless Humidity And Temperature
 *
 * Copyright (C) 2017 Daniel Collins <solemnwarning@solemnwarning.net>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the author nor the names of its contributors may
 *     be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ip_addr.h>
#include <mem.h>
#include <os_type.h>
#include <osapi.h>
#include <espconn.h>

#define DEBUG_TCP_TIMEOUT 60
#define DEBUG_LOG_SIZE    1024

/* clients = espconn
 *     espconn->reverse = conn_state
 *        conn_state->next = espconn
 *        ....
*/

struct conn_state {
	char *log_read_ptr;
	bool ready_to_send;
	
	struct espconn *next;
};

static struct espconn *listener = NULL;
static struct espconn *clients  = NULL;

/* log_read_ptr:  Address to read from for full log
 * log_write_ptr: Next address to write to
 * 
 * Buffer empty:
 *   log_read_ptr  = NULL
 *   log_write_ptr = log_buffer
 * 
 * Buffer partially filled:
 *   log_read_ptr  = log_buffer
 *   log_write_ptr = log_buffer + x
 * 
 * Buffer full:
 *   log_read_ptr  = log_buffer + x
 *   log_write_ptr = log_read_ptr
 * 
 * Each connection has its own read pointer, which indicates where the next
 * read should start. When the client's read pointer is on the write pointer
 * there is nothing waiting to send. When its NULL, we haven't started sending
 * the log to the client and we start from the global read pointer.
*/

static char log_buffer[DEBUG_LOG_SIZE];

static char        *log_read_ptr  = NULL;
static char        *log_write_ptr = log_buffer;
static char * const log_end_ptr   = log_buffer + sizeof(log_buffer);

static void _debug_accept(void *arg);
static void _debug_sent(void *arg);
static void _debug_close(void *arg);
static void _debug_putc(char c);
static void _debug_flush_all(void);
static void _debug_flush(struct espconn *conn);

/* Setup the debug console listening port (if enabled) */
void ICACHE_FLASH_ATTR debug_init(uint16_t port)
{
	listener = os_zalloc(sizeof(struct espconn) + sizeof(esp_tcp));
	if(!listener)
	{
		return;
	}
	
	listener->type                  = ESPCONN_TCP;
	listener->state                 = ESPCONN_NONE;
	listener->proto.tcp             = (esp_tcp*)(listener + 1);
	listener->proto.tcp->local_port = port;
	
	espconn_regist_connectcb(listener, &_debug_accept);
	espconn_accept(listener);
	espconn_regist_time(listener, DEBUG_TCP_TIMEOUT, 0);
	
	os_install_putc1(&_debug_putc);
}

/* Callback which accepts a connection on the debug port */
static void ICACHE_FLASH_ATTR _debug_accept(void *arg)
{
	struct espconn *conn = (struct espconn*)(arg);
	
	struct conn_state *cs = os_malloc(sizeof(struct conn_state));
	if(!cs)
	{
		static const char *oom_msg = "Out of memory. Not attached to console.\n";
		espconn_send(conn, (uint8*)(oom_msg), os_strlen(oom_msg));
		
		return;
	}
	
	cs->log_read_ptr  = NULL;
	cs->ready_to_send = true;
	cs->next          = clients;
	
	conn->reverse = cs;
	clients = conn;
	
	espconn_regist_sentcb(conn, &_debug_sent);
	espconn_regist_disconcb(conn, &_debug_close);
	
	_debug_flush(conn);
}

static void ICACHE_FLASH_ATTR _debug_sent(void *arg)
{
	struct espconn *conn  = (struct espconn*)(arg);
	struct conn_state *cs = conn->reverse;
	
	cs->ready_to_send = true;
	
	_debug_flush(conn);
}

/* Callback which removed a client from the list when it disconnects. */
static void ICACHE_FLASH_ATTR _debug_close(void *arg)
{
	struct espconn *conn = (struct espconn*)(arg);
	
	struct espconn **nptr = &clients;
	while(*nptr)
	{
		struct conn_state *cs = (*nptr)->reverse;
		
		if(os_memcmp((*nptr)->proto.tcp, conn->proto.tcp, sizeof(esp_tcp)) == 0)
		{
			*nptr = cs->next;
			os_free(cs);
			break;
		}
		
		nptr = &(cs->next);
	}
}

/* putc() hook which puts the log from os_printf() in the debug log. */
static void ICACHE_FLASH_ATTR _debug_putc(char c)
{
	/* Write to the buffer and advance the read/write pointers. */
	
	*log_write_ptr = c;
	
	if(log_read_ptr == NULL)
	{
		log_read_ptr = log_buffer;
	}
	else if(log_read_ptr == log_write_ptr)
	{
		++log_read_ptr;
	}
	
	if(++log_write_ptr == log_end_ptr)
	{
		log_read_ptr = log_write_ptr = log_buffer;
	}
	
	/* Iterate over the connections and force any whose read pointer we've
	 * just stepped onto to restart from the global read pointer when they
	 * are ready to send.
	 * 
	 * This sucks. They will lose some bytes out of the backlog, but at
	 * least its unlikely unless they stop sending ACKs for long enough or
	 * the console is being updated so quickly that the buffer can fully
	 * cycle between ACKs.
	 * 
	 * An alternative would be to disconnect, but that is more complicated
	 * since we could be in the context of an espconn callback where trying
	 * to disconnect the connection would wander into undefined behaviour...
	*/
	
	struct espconn *conn = clients;
	while(conn)
	{
		struct conn_state *cs = conn->reverse;
		
		if(cs->log_read_ptr == log_write_ptr)
		{
			cs->log_read_ptr = NULL;
		}
		
		conn = cs->next;
	}
	
	/* Trigger any new sends */
	
	_debug_flush_all();
}

static void ICACHE_FLASH_ATTR _debug_flush_all(void)
{
	struct espconn *conn = clients;
	while(conn)
	{
		struct conn_state *cs = conn->reverse;
		
		_debug_flush(conn);
		
		conn = cs->next;
	}
}

static void ICACHE_FLASH_ATTR _debug_flush(struct espconn *conn)
{
	struct conn_state *cs = conn->reverse;
	
	if(cs->ready_to_send && cs->log_read_ptr != log_write_ptr)
	{
		if(cs->log_read_ptr == NULL)
		{
			/* Sending the first chunk after connection, we start
			 * sending from the global read pointer.
			 * 
			 * This needs to be a special case as the read pointer
			 * may also be the write pointer, where we we normally
			 * don't send anything.
			*/
			
			if(log_read_ptr == log_write_ptr)
			{
				/* Buffer is full, so we send from the read
				 * pointer to the end of it and place our read
				 * pointer at the start of the buffer to
				 * continue up to the write pointer next time.
				*/
				
				espconn_send(conn, (uint8*)(log_read_ptr), (log_end_ptr - log_read_ptr));
				cs->log_read_ptr = log_buffer;
			}
			else{
				/* Buffer isn't full, send from the global read
				 * pointer which will be positioned at the start
				 * to the write pointer signifying the end of
				 * the current backlog.
				*/
				
				espconn_send(conn, (uint8*)(log_read_ptr), (log_write_ptr - log_read_ptr));
				cs->log_read_ptr = log_write_ptr;
			}
		}
		else if(cs->log_read_ptr > log_write_ptr)
		{
			/* Our read pointer is later in the buffer then the
			 * write pointer, read up to the end of it and then our
			 * next send will be from the start to the write
			 * pointer.
			*/
			
			espconn_send(conn, (uint8*)(cs->log_read_ptr), (log_end_ptr - cs->log_read_ptr));
			cs->log_read_ptr = log_buffer;
		}
		else{
			/* Send from our read pointer up to the write pointer. */
			
			espconn_send(conn, (uint8*)(cs->log_read_ptr), (log_write_ptr - cs->log_read_ptr));
			cs->log_read_ptr = log_write_ptr;
		}
		
		cs->ready_to_send = false;
	}
}
