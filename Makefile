CC = arm-none-eabi-gcc
AR = arm-none-eabi-ar

CFLAGS = -Wall -g -O2 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections

SRC	= src/driver_mpu6050.c

OBJ = $(SRC:.c=.o)
STATIC_LIB = libmpu6050.a

all: $(STATIC_LIB)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(STATIC_LIB): $(OBJ)
	$(AR) rcs $(STATIC_LIB) $(OBJ)

clean:
	rm -f $(OBJ) $(STATIC_LIB)