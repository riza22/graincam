#ifndef GPIO_H
#define GPIO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#define GPIO_OUT	1
#define GPIO_IN		0
#define GPIO_HI		1
#define GPIO_LO		0
#define MAX_BUF 	64
#define SYSFS_GPIO_DIR "/sys/class/gpio"

int gpio_export(unsigned int gpio);
int gpio_unexport(unsigned int gpio);
int gpio_set_dir(unsigned int gpio, unsigned int out_flag);
int gpio_set_value(unsigned int gpio, unsigned int value);
int gpio_get_value(unsigned int gpio, unsigned int *value);
int gpio_set_edge(unsigned int gpio, char *edge);
int gpio_fd_open(unsigned int gpio);
int gpio_fd_close(int fd);

#endif
