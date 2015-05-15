#OpenNI
sudo apt install libopenni-dev
-----------------------------------------------------------------------------------------
#PrimeSense 

sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
git clone https://github.com/avin2/SensorKinect.git

Building Sensor:

1. Go into the directory: "Platform/Linux/CreateRedist".
	   Run the script: "./RedistMaker".

	   This will compile everything and create a redist package in the "Platform/Linux/Redist" directory.
	   
	   It will also create a distribution in the "Platform/Linux/CreateRedist/Final" directory.
1. Go into the directory: "Platform/Linux/Redist".
	   Run the script: "sudo ./install.sh" (needs to run as root)

-----------------------------------------------------------------------------------------

#OpenCV with OpenNI 
Use this link to install OpenCV Dependencies in your linux machine.

http://www.samontab.com/web/2014/06/installing-opencv-2-4-9-in-ubuntu-14-04-lts/

1. cd opencv-2.4.9
1. mkdir build
1. cd build
1. cmake -DCMAKE_BUILD_TYPE=DEBUG -DBUILD_SHARED_LIBS=ON -DBUILD_PYTHON_SUPPORT=ON -DBUILD_ZLIB=ON -DBUILD_TIFF=ON -DBUILD_JASPER=ON -DBUILD_JPEG=ON -DBUILD_PNG=ON -DBUILD_OPENEXR=ON -DWITH_TBB=ON -DWITH_UNICAP=ON -DWITH_FFMPEG=ON -DWITH_OPENNI=ON ..
1. Check that it detects the OpenCV and PrimeSense Library correctly
1. make
1. sudo make install

--------------------------------------------------------------------------------------------------------

#PCL Library 

sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all

git clone https://github.com/avin2/SensorKinect.git
Building Sensor:
	1. Go into the directory: "Platform/Linux/CreateRedist".
	   Run the script: "./RedistMaker".
	   This will compile everything and create a redist package in the "Platform/Linux/Redist" directory.
	   It will also create a distribution in the "Platform/Linux/CreateRedist/Final" directory.
	2. Go into the directory: "Platform/Linux/Redist".
	   Run the script: "sudo ./install.sh" (needs to run as root)



--------------------------------------------------------------------------------------------------------

#Libpointmatcher 

1. Download the source code from https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Compilation.md. 
2. Follow the instructions there to install dependencies and libpointmatcher
3. Copy files from /PATH HERE/ to /PATH HERE/
4. Recompile libpointmatcher
