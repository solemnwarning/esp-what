CC = xtensa-lx106-elf-gcc
CFLAGS = -Wall -Wno-implicit-function-declaration -I./src/ -mlongcalls -std=c99 -DICACHE_FLASH
LDLIBS = -nostdlib -Wl,--start-group -lmain -lnet80211 -lwpa -llwip -lpp -lphy -Wl,--end-group -lgcc -lc
LDFLAGS = -Teagle.app.v6.ld

CORE_SRCS := src/main.c src/num.c src/debug.c

DHT11_SRCS := $(CORE_SRCS) src/dht.c
DHT11_OBJS := $(addsuffix .dht11.o,$(basename $(DHT11_SRCS)))

DHT22_SRCS := $(CORE_SRCS) src/dht.c
DHT22_OBJS := $(addsuffix .dht22.o,$(basename $(DHT22_SRCS)))

.PHONY: all
all: what-dht11-0x00000.bin what-dht22-0x00000.bin

.PHONY: clean
clean:
	rm -f what-dht11-0x00000.bin what-dht11-0x10000.bin what-dht11 $(DHT11_OBJS)
	rm -f what-dht22-0x00000.bin what-dht22-0x10000.bin what-dht22 $(DHT22_OBJS)

what-dht11-0x00000.bin what-dht11-0x10000.bin: what-dht11
	esptool.py elf2image $^

what-dht11: $(DHT11_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

%.dht11.o: %.c src/what.h
	$(CC) -DSENSOR_TYPE_DHT11 $(CFLAGS) -c -o $@ $<

what-dht22-0x00000.bin what-dht22-0x10000.bin: what-dht22
	esptool.py elf2image $^

what-dht22: $(DHT22_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

%.dht22.o: %.c src/what.h
	$(CC) -DSENSOR_TYPE_DHT22 $(CFLAGS) -c -o $@ $<
