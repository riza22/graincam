#include "gpio.h"
#include "spi.h"
#include <poll.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <cv.h>


#define COM_3			130
#define FINE_FRAME		132
#define STOP			133
#define FINE_RIGA		134



#define RESET_FPGA		131
#define SCARICA_RIGA	138
#define CC_SPI			139

#define GPIO_DATAIN		gpio[0x6038/4]
#define GPIO_DATAOUT	gpio[0x603C/4]


#define M_COM_3			GPIO_DATAOUT & 0x00000004
#define M_COM_2			GPIO_DATAOUT & 0x00000008
#define M_FINE_FRAME	GPIO_DATAIN & 0x00000010
#define M_STOP			GPIO_DATAIN & 0x00000020
#define M_FINE_RIGA		GPIO_DATAIN & 0x00000040
#define M_CC_SPI		GPIO_DATAOUT & 0x00000080
#define M_RESET_FPGA	GPIO_DATAOUT & 0x00000400
#define M_SCARICA_RIGA	GPIO_DATAOUT & 0x00000800

void init_gpio() {
    //gpio_export(COM_1);
    /*	gpio_export(COM_3);*/
    /*	gpio_export(COM_2);*/
    /*	gpio_export(SCARICA_RIGA);*/
    /*	gpio_export(CC_SPI);*/
    /*	gpio_export(RESET_FPGA);*/
    /*	gpio_export(FINE_FRAME);*/
    /*	gpio_export(STOP);*/
    /*	gpio_export(FINE_RIGA);*/

    //	gpio_set_dir(COM_1,GPIO_OUT);
    gpio_set_dir(COM_3, GPIO_OUT);
    //gpio_set_dir(COM_2,GPIO_OUT);
    gpio_set_dir(SCARICA_RIGA, GPIO_OUT);
    gpio_set_dir(CC_SPI, GPIO_OUT);
    gpio_set_dir(RESET_FPGA, GPIO_OUT);
    gpio_set_dir(FINE_FRAME, GPIO_IN);
    gpio_set_dir(STOP, GPIO_IN);
    gpio_set_dir(FINE_RIGA, GPIO_IN);


}

void wait(int gpio, char * type) {
    char buf[64];
    int fd;
    struct pollfd fdset[1];
    int len, rc;

    fd = gpio_fd_open(gpio);
    gpio_set_edge(gpio, type);
    memset((void*) fdset, 0, sizeof (fdset));
    fdset[0].fd = fd;
    fdset[0].events = POLLPRI;
    while (1) {
        rc = poll(fdset, 1, -1);
        if (rc < 0) {
            perror("poll failed\n");
        }
        if (fdset[0].revents & POLLPRI) {
            //	return;
            lseek(fdset[0].fd, 0, SEEK_SET);
            len = read(fdset[0].fd, &buf, MAX_BUF);
            if ((type == "rising") && (buf[0] == '1')) {
                return;
            } else if ((type == "faling") && (buf[0] = '0')) {
                return;
            }
        }

    }
}

void wait3() {
    FILE * fs;
    fd_set readSet;
    struct timeval tv;
    int rc, fd;
    printf("open\n");
    fd = gpio_fd_open(FINE_RIGA);
    printf("reset\n");
    FD_ZERO(&readSet);
    FD_SET(fd, &readSet);
    char argStr[ 60 ];
    ssize_t numBytes;

    printf("wait\n");
    rc = select(fd + 1, &readSet, NULL, NULL, NULL);
    printf("after wait\n");
}

void wait2() {
    FILE * fs;
    fd_set readSet;
    struct timeval tv;
    int rc;
    printf("open\n");
    if ((fs = fopen("/dev/gpio-event", "r")) == NULL) {
        printf("Check to make sure gpio_event_drv has been loaded. Unable to open /dev/gpio-event");
        exit(1);
    }
    printf("reset\n");
    FD_ZERO(&readSet);
    FD_SET(fileno(fs), &readSet);
    char argStr[ 60 ];
    ssize_t numBytes;

    printf("wait\n");
    rc = select(fileno(fs) + 1, &readSet, NULL, NULL, NULL);
    printf("after wait\n");
}

void reset_gpio(int gpio) {
    gpio_set_value(gpio, GPIO_LO);
    usleep(10);
    gpio_set_value(gpio, GPIO_HI);
}

volatile ulong *gpio;

int init_gpio_mem() {
    int fd;
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    // GPIO Configuration: configure are input
    gpio = (ulong*) mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x49050000);
    if (gpio == MAP_FAILED) {
        printf("Gpio Mapping failed\n");
        close(fd);
        return 0;
    }
    return 1;
}

void reset_ccspi() {
    GPIO_DATAOUT = GPIO_DATAOUT & 0xFFFFF7FF;
    GPIO_DATAOUT = GPIO_DATAOUT | 0x00000800;
}

void reset_txor() {
    GPIO_DATAOUT = GPIO_DATAOUT & 0xFFFFFBFF;
    GPIO_DATAOUT = GPIO_DATAOUT | 0x00000400;
}

void reset_fpga() {
    GPIO_DATAOUT = GPIO_DATAOUT & 0xFFFFFFF7;
    usleep(1);
    GPIO_DATAOUT = GPIO_DATAOUT | 0x00000008;
}

void reset_com3() {
    GPIO_DATAOUT = GPIO_DATAOUT & 0xFFFFFFFB;
    GPIO_DATAOUT = GPIO_DATAOUT | 0x00000004;
}

char *int2bin(unsigned n, char *buf) {
#define BITS 8
    //(sizeof(n) * CHAR_BIT)

    static char static_buf[BITS + 1];
    int i;

    if (buf == NULL)
        buf = static_buf;

    for (i = BITS - 1; i >= 0; --i) {
        buf[i] = (n & 1) ? '1' : '0';
        n >>= 1;
    }

    buf[BITS] = '\0';
    return buf;

#undef BITS
}

IplImage * convert_img(char * buffer) {
    IplImage* img = cvCreateImage(cvSize(128, 64), IPL_DEPTH_8U, 1);
    uchar* data = (uchar *) img->imageData;
    int step = img->widthStep / sizeof (uchar);
    int i, j, ctr2 = 0;
    for (i = 0; i < 64; i++) {
        for (j = 0; j < 128; j += 8) {
            data[i * step + j + 0] = 255 * (buffer[ctr2]&0x80) >> 7;
            data[i * step + j + 1] = 255 * (buffer[ctr2]&0x40) >> 6;
            data[i * step + j + 2] = 255 * (buffer[ctr2]&0x20) >> 5;
            data[i * step + j + 3] = 255 * (buffer[ctr2]&0x10) >> 4;
            data[i * step + j + 4] = 255 * (buffer[ctr2]&0x08) >> 3;
            data[i * step + j + 5] = 255 * (buffer[ctr2]&0x04) >> 2;
            data[i * step + j + 6] = 255 * (buffer[ctr2]&0x02) >> 1;
            data[i * step + j + 7] = 255 * (buffer[ctr2]&0x01);
            ctr2++;
        }
    }
    /*	unsigned temp;
            for (i=0;i<8192<;i++){
                    if ((i!=0) && (i%8=0)) ctr2++;
                    temp=buffer[ctr2];
                    data[i]=(temp & 1);
                    temp>>=1;
            }
     */
    return img;
}

char buffer[1024];
uint8_t tx[1] = {0xC8};
int8_t rx[1] = {0x00};
uint8_t rx1, tx1 = 0x00;
int ctr = 0;
uint8_t tx2[8]={0,0,0,0,0,0,0,0};
uint8_t rx2[8]={0,0,0,0,0,0,0,0};

inline void read_row() {
    int i;
    for (i = 0; i < 16; i++) {
        buffer[ctr++] = read_spi();
        //readn_spi(tx1,&rx1);

        //    readn_spib(tx, rx, 8);

        //  readn_spib(tx, rx, 8);

        //readn_spi(8);
    }
}

inline void read_row2() {
    int i;
    readn_spib(tx2, rx2, 8);
    for (i=0;i<8;i++){
        buffer[ctr++]=rx2[i];
    }
    readn_spib(tx2, rx2, 8);
    for (i=0;i<8;i++){
        buffer[ctr++]=rx2[i];
    }

}

int main(int args, char ** argv) {
    uint8_t integration_time = 0x19;
    if (args > 1) {
        integration_time = atoi(argv[1]);
    } else {
        integration_time = 10;
    }
    uint8_t tx[1] = {0xC8};
    uint8_t rx[1] = {0x00};
    int i, j, temp;
    //char C[520]={0xFF,};
    int fd;

    init_spi2(0, 8, 48000000, 0);
    init_gpio();
    init_gpio_mem();

    char frame_name[20];
    IplImage* img;
    int id_frame = 0;
    int N = 2;
    cvNamedWindow("image",0);
    cvResizeWindow("image",128*4,64*4);

    // initialize the buffer
    for (i = 0; i < 1024; i++) {
        buffer[i] = 0;
    }
    reset_txor();
    reset_fpga();
    reset_com3();
    //write_spi(0x19); // 5 ms integration time;
    write_spi(integration_time);
    GPIO_DATAOUT = GPIO_DATAOUT & 0xFFFFFFFB;
    usleep(500);
    while (1) {
        ctr = 0;
        //GPIO_DATAOUT = GPIO_DATAOUT & 0xFFFFFF7F; // cc_spi=0
        //GPIO_DATAOUT = GPIO_DATAOUT & 0xFFFFFFFB; // com_3 =0
        //GPIO_DATAOUT = GPIO_DATAOUT | 0x00000800; // txor =1;

        reset_txor();
        reset_fpga();
        reset_ccspi();

        while (1) {
            if (M_STOP) break;
        }
        usleep(150);
        read_row2();
        ctr += 16;
        for (j = 0; j < 31; j++) {
            reset_ccspi();
            reset_txor();
            read_row2();
            ctr += 16;
        }
        reset_txor();
        usleep(180);
        ctr = 16;
        for (j = 0; j < 32; j++) {
            reset_ccspi();
            reset_txor();
            read_row2();
            ctr += 16;
        }
        ///*
        // print the image as text
        /*
                        for (i=0;i<1024;i++){
                                if ((i!=0)&&(i%16==0)) printf("\n");
                                printf("%s",int2bin(buffer[i], NULL));
                        }

                        printf("\n");
         */
        /*		printf("ctr : %d\n",ctr);*/
        //printf("frame id:%d\n",id_frame);
        //*/

        img = convert_img(buffer);
        cvShowImage("image", img); 
        cvSaveImage(filename,img);
        cvWaitKey(5);
        //id_frame++;
    }
    cvReleaseImage(&img);
    return 0;
}
