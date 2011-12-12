#ifndef USER_SPI
#define USER_SPI

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define SPI_DEVICE "/dev/spidev4.0"
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

int init_spi();
int init_spi2(uint8_t mode, uint8_t bits,uint32_t speed,uint16_t delay);
void close_spi();
void write_read_spi(uint8_t * tx, uint8_t * rx);
void write_spi(uint8_t val);
uint8_t read_spi();
int read_spi16(uint8_t * rx);
#endif
