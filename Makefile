CXXFLAGS =	-O2 -g -Wall -std=c++0x -fmessage-length=0

OBJS =		tracking-robot.o StepperMotor.o

LIBS =		-lopencv_core -lopencv_contrib -lopencv_highgui -lraspicam -lraspicam_cv -lopencv_objdetect -lopencv_imgproc -lwiringPi

TARGET =	tracking-robot

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
