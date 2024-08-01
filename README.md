# Time-of-flight Vision System


### *Objective*
The objective of this project is to estimate products' orientation and dimensions using a time-of-flight (ToF) camera. 
The project is made as a Robotics Engineering Bachelor project, in collaboration with a warehouse logistics company.
The company has a solution that already performs this task, but since it is quit expensive, cumbersome, and a closed third-party vendor system they would like to explore the possibility of using a ToF camera instead. Since it's cheaper and they can customize the software themselve. 

### *How to get it running*
This application was build to run on the Ubuntu (Linux) operating system.
The camera used for this project is a ifm electronics O3D303 (https://www.ifm.com/gb/en/product/O3D303) and ifm provides a developer C++ library, which is used in this project. This means that there are some step that must be taken in order to be able to run the application.

1. Go to "https://github.com/ifm/ifm3d", download the repository and follow the installation guide (ifm C++ developer library).
2. Go to "https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html?ref=wasyresearch.com", and follow the instructions to install OpenCV.
3. Download this repository.
4. Open a terminal and navigate to the parent folder aka "Time-of-Flight_Vision_System-master" and run the following commands:
   1. $ make
   2. $ ./main

Now the application should be running.

### *How to use to application*
In progress...

### *License* 
This project is licensed under the MIT License - see the [LICENSE.md](https://github.com/Svendsen92/Machine_Vision/blob/master/LICENSE.md) file for details.
