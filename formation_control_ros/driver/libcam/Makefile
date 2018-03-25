CC = g++

TARGET = libcam.so

OBJS = withrobot_utility.o \
	withrobot_camera.o

INCS	+= -I./include
INCS    += -I./lib/include
INCS	+= -I./usr/include

LIBS	+= -lpthread
LIBS	+= -ludev
LIBS	+= -lv4l2
OPENCV_CFLAGS  = $(shell pkg-config --cflags opencv)
OPENCV_LIBS	   = $(shell pkg-config --libs opencv)

LDFLAGS     = $(LIBS) $(OPENCV_LIBS)

CXXFLAGS = $(INCS) $(OPENCV_CFLAGS) -c -std=c++11 -g  -fPIC -W -Wfatal-errors #-Wall

RM 			= @rm -rfv

MKDIR			= @mkdir -p

$(BIN_PATH)/$(TARGET): $(OBJS)
	$(CC) -shared -o $(TARGET) $(OBJS) $(LDFLAGS)


withrobot_utility.o: withrobot_utility.cpp
	$(CC) $(CXXFLAGS) $< -o $@

withrobot_camera.o: withrobot_camera.cpp
	$(CC) $(CXXFLAGS) $< -o $@



clean:
	
	$(RM) $(OBJS) $(TARGET)
install:
	$(MKDIR) /usr/local/include/ocam/
	cp ./$(TARGET) /usr/local/lib
	cp ./*.hpp /usr/local/include/ocam
