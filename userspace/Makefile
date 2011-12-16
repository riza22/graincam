CFLAGS = -Wall -pthread -I/usr/include/opencv
CXXFLAGS = -I/usr/include/opencv
LDFLAGS = -pthread
LDLIBS = -lrt -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann

APP=main
OBJS=main.o gpio.o spi.o graincam.o

$(APP): $(OBJS)

clean:
	rm -f *.d *.o $(APP) *~

###################################
%.d: %.c
	$(CC) -MM -MF $@ $<

%.o: %.d

-include $(OBJS:.o=.d)

