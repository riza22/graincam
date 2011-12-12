CXX=g++
CC=gcc
CFLAGS=-Wall -pthread -O3 -I/usr/include/opencv
LDFLAGS=-lrt -lopencv_core -lopencv_highui 
SOURCES= kalman.cpp common.cpp periodic.c
CXX_OBJECTS=$(SOURCES:.cpp=.o)
CC_OBJECTS=$(SOURCES:.c=.o)

all: graincam_test
	
#$(EXECUTABLE1): $(CXX_OBJECTS) 
#	$(CXX) $(LDFLAGS) $(CFLAGS) $(CXX_OBJECTS) -o $@

#$(EXECUTABLE2): $(CC_OBJECTS) 
#	$(CC) $(LDFLAGS) $(CFLAGS) $(CC_OBJECTS) -o $@


graincam_test: graincam_test.o 
	$(CC) -L/usr/lib -o"graincam_test" graincam_test.o  -lopencv_highgui -lopencv_core -lrt
	
.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o graincam_test *.d
