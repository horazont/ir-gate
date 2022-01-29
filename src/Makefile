CC = avr-gcc
CFLAGS ?= -Wall -Wextra -Werror -Os -std=gnu11
CFLAGS += -mmcu=attiny2313a -fstack-usage -Wstack-usage=64 -fdata-sections -ffunction-sections -Wl,--gc-sections

OBJCOPY = avr-objcopy
OCFLAGS ?= --strip-unneeded

SIZE = avr-size

PROGRAM = main
SRCS = main.c
COMMONDEPS = 

AVRDUDE=avrdude
AVRDUDEFLAGS=-b 115200
AVRISP=avrisp
SERIAL=/dev/ttyUSB0
AVR=t2313

OBJS = $(addsuffix .o,$(basename $(SRCS)))
PRODUCTS = $(OBJS) $(PROGRAM).hex $(PROGRAM).elf

all: $(PROGRAM).hex

$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) $(OCFLAGS) -O ihex $< $@

$(PROGRAM).elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@
	@echo -e "\nbinary size:"
	@$(SIZE) $@
	@echo

flash: $(PROGRAM).hex
	$(AVRDUDE) $(AVRDUDEFLAGS) -p $(AVR) -c $(AVRISP) -P $(SERIAL) -U "flash:w:$(PROGRAM).hex"

allstrings.h: strings.list utils/mkstrings.py
	./utils/mkstrings.py $< > $@

$(OBJS): %.o: %.c $(COMMONDEPS)
	$(CC) -c $(CFLAGS) $< -o $@

%.s: %.c $(COMMONDEPS)
	$(CC) -S -fverbose-asm -c $(CFLAGS) $< -o $@

.PHONY: clean flash

clean:
	rm -f $(PRODUCTS)
