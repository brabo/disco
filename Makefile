MAIN = main.c
LOCAL_SOURCE_DIR = ./src/
LOCAL_SOURCE = printf.c pwm.c usart.c i2c.c rtc.c clock.c init.c web.c ff.c fileio.c sd_spi_loc3_stm32.c fattime.c cdb_unpack.c cdb_seek.c cdb_hash.c msgpack.c

ROOT = .
SRC_DIR = ./src/

TOOLROOT=/usr/local/arm/bin

NETIF_SOURCE_DIR = ./net/
NETIF_SOURCE = pico_stm32f107.c

INC = ./inc/
PICO_INCLUDE = ./testpico/build/include/
PICO_LIB = ./testpico/build/lib/
PICO_HTTP = ./picotcp-modules/libhttp

NEWLIB_INCLUDE = /usr/local/arm-none-eabi/include
NEWLIB_LIB = /usr/local/arm-none-eabi/lib

# Tools

CC=$(TOOLROOT)/arm-none-eabi-gcc
LD=$(TOOLROOT)/arm-none-eabi-gcc
GDB=/usr/local/bin/arm-none-eabi-gdb
AR=$(TOOLROOT)/arm-none-eabi-ar
AS=$(TOOLROOT)/arm-none-eabi-as
OPENOCD=/usr/local/bin/openocd
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size


TARGET = $(strip $(basename $(MAIN)))
SRC = $(addprefix $(LOCAL_SOURCE_DIR), $(TARGET).c)

EXTRA_SOURCE =  $(addprefix $(LOCAL_SOURCE_DIR), $(LOCAL_SOURCE))
EXTRA_SOURCE += $(addprefix $(NETIF_SOURCE_DIR), $(NETIF_SOURCE))

SRC += $(EXTRA_SOURCE)
OBJS = $(SRC:.c=.o)


CFLAGS = -mcpu=cortex-m3 -mthumb
CFLAGS += -Wall -g -O3
CFLAGS += -fno-tree-loop-distribute-patterns -fno-common
CFLAGS += -I$(INC) -I$(NETIF_SOURCE_DIR) -I$(NEWLIB_INCLUDE) -I$(PICO_INCLUDE) -I$(PICO_HTTP)
CFLAGS += -L$(NEWLIB_LIB) -L$(PICO_LIB) -L$(PICO_HTTP)
CFLAGS += -MD -DSTM32F1 -DSTM32
CFLAGS += -lopencm3_stm32f1 -lpicotcp -lhttp -lm -lc -lnosys
LDFLAGS = --static -fno-tree-loop-distribute-patterns
LDFLAGS += -T ld/libopencm3_stm32f1.ld -nostartfiles -Wl,--gc-sections -Wl,-Map=$(TARGET).map
#CFLAGS+=-Wall -g -fno-tree-loop-distribute-patterns -fno-common -mcpu=cortex-m3 -mthumb -MD -DSTM32F1 -I/usr/local/arm-none-eabi/include -DSTM32 -I./testpico/build/include/ -I./inc -L./testpico/build/lib/ -I./picotcp-modules/libhttp -L./picotcp-modules/libhttp -L/usr/local/arm-none-eabi/lib -I$(NETIF_SOURCE_DIR) -L$(NETIF_SOURCE_DIR) -I./inc -lopencm3_stm32f1
#CFLAGS +=-I./testpico/build/include/ -L./testpico/build/lib/ -DPRINTF_SUPPORT_LONG
#LDFLAGS=--static -lc -T ld/stm32_ram.ld -nostartfiles -Wl,--gc-sections -nostdlib -mcpu=cortex-m3 -mthumb -lm -Wl,-Map=tut.map


flash: all
	$(GDB) main.elf

all: main.hex

main.hex: main.elf
	$(OBJCOPY) -R .eeprom -O ihex main.elf main.hex

main.elf: $(OBJS)
	$(CC) -o $(TARGET).elf $(OBJS) $(LDFLAGS) $(CFLAGS)

#$(OPENOCD) -f openocd.cfg -c "init" -c "reset init" -c "flash write_image erase main.hex 0x08000000" -c "reset run" -c "shutdown"

clean:
	rm -f *.elf *.bin *.list *.map *.o *.d *~ *.hex
	rm -f src/*.o

test:
	gcc -o test t/01_c.c src/pwm.c -I./inc -DSTM32 -DSTM32F1 -T ld/libopencm3_stm32f1.ld -I/usr/local/arm-none-eabi/include -L/usr/local/arm-none-eabi/lib -lopencm3_stm32f1
	./test
