
CXX_INCLUDES = -isystem /usr/include/opencv -isystem /usr/include/pcl-1.7 -isystem /usr/include/eigen3


OpenCV_FLAGS = /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9 /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9 /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9

IFM3D_FLAGS = /usr/lib/libifm3d_image.so.0.11.2 /usr/lib/libifm3d_framegrabber.so.0.11.2 /usr/lib/libifm3d_camera.so.0.11.2 /usr/lib/x86_64-linux-gnu/libglog.so 

LD_FLAGS = -lpthread -lstdc++ -lm

CFLAGS = -Wall -std=c++14 -Iinclude -c -o

all:
	gcc $(CFLAGS) objFiles/main.o src/main.cpp $(CXX_INCLUDES)
	echo "This is the first command"
	gcc $(CFLAGS) objFiles/config.o src/config.cpp $(CXX_INCLUDES)
	echo "This is the second command"
	gcc $(CFLAGS) objFiles/imgProcess.o src/imgProcess.cpp
	echo "This is the third command"
	gcc $(CFLAGS) objFiles/geometricalData.o src/geometricalData.cpp
	echo "This is the forth command"
	gcc $(CFLAGS) objFiles/imgConstruct.o src/imgConstruct.cpp
	
	echo "This is the final command"
	gcc -Wall -std=c++14 -o main objFiles/main.o objFiles/config.o objFiles/imgProcess.o objFiles/geometricalData.o objFiles/imgConstruct.o $(OpenCV_FLAGS) $(IFM3D_FLAGS) $(LD_FLAGS)
	echo "The project has been successfully compiled"

clean:
	rm main
	rm objFiles/main.o
	rm objFiles/config.o
	rm objFiles/imgProcess.o
	rm objFiles/geometricalData.o
	rm objFiles/imgConstruct.o 

	

