CXX=g++
CC=gcc
CFLAGS=-Wall -pthread -O3 -I/usr/include/opencv
LDFLAGS=-lrt -lopencv_core -lopencv_highui 
CXX_OBJECTS=$(SOURCES:.cpp=.o)
CC_OBJECTS=$(SOURCES:.c=.o)

all: graincam_test

graincam_test: graincam_test.o gpio.o spi.o
	$(CC) -L/usr/lib -o"graincam_test" graincam_test.o gpio.o spi.o -lopencv_highgui -lopencv_core -lrt
	
.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o graincam_test *.d
