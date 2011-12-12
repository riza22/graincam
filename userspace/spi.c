#include "spi.h"

static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 2000000;
static uint16_t delay = 0;
int spi_fd;

int init_spi(){
	int ret = 0;
	spi_fd = open(SPI_DEVICE, O_RDWR);
	if (spi_fd<0) perror("can't open device");
	ret = ioctl(spi_fd, SPI_IOC_WR_MODE,&mode);
	if (ret==-1) perror("can't set spi mode");
	ret = ioctl(spi_fd, SPI_IOC_RD_MODE,&mode);
	if (ret==-1) perror("can't get spi mode");
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1) perror("can't set bits per word");
    ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1) perror("can't get bits per word");
	ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1) perror("can't set max speed hz");
    ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1) perror("can't get max speed hz");
	return spi_fd;
}
int init_spi2(uint8_t mode, uint8_t bits,uint32_t speed,uint16_t delay){
	int ret = 0;
	spi_fd = open(SPI_DEVICE, O_RDWR);
	if (spi_fd<0) perror("can't open device");
	ret = ioctl(spi_fd, SPI_IOC_WR_MODE,&mode);
	if (ret==-1) perror("can't set spi mode");
	ret = ioctl(spi_fd, SPI_IOC_RD_MODE,&mode);
	if (ret==-1) perror("can't get spi mode");
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1) perror("can't set bits per word");
    ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1) perror("can't get bits per word");
	ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1) perror("can't set max speed hz");
    ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1) perror("can't get max speed hz");
	return spi_fd;
}

void close_spi(){
	close(spi_fd);
}


void write_read_spi(uint8_t * tx, uint8_t * rx){
	int ret =0;
	struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = ARRAY_SIZE(tx),
                .delay_usecs = delay,
                .speed_hz = speed,
                .bits_per_word = bits,
	};
	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) perror("can't send spi message");
}

void write_spi(uint8_t val){
	int ret =0;
	uint8_t tx[1]={val};
	uint8_t rx[1]={0x00};
	struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = 1,
                .delay_usecs = delay,
                .speed_hz = speed,
                .bits_per_word = bits,
	};
	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) perror("can't send spi message");
}

uint8_t read_spi(){
	int ret =0;
	uint8_t tx[1]={0x00};
	uint8_t rx[1]={0x00};
	struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = 1,
                .delay_usecs = delay,
                .speed_hz = speed,
                .bits_per_word = bits,
	};
	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) perror("can't send spi message");
	return rx[0];
}


int read_spi16(uint8_t * rx){
	int ret =0;
	struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)NULL,
                .rx_buf = (unsigned long)rx,
                .len = 16,
               	.delay_usecs = 20,
              //  .speed_hz = speed,
              //  .bits_per_word = bits,
	};
	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
		perror("can't send spi message");
		return -1;
	}
	return 0;
}
