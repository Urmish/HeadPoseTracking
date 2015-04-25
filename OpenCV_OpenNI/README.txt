OpenNI -

sudo apt install libopenni-dev
-----------------------------------------
PrimeSense -

sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all

git clone https://github.com/avin2/SensorKinect.git
Building Sensor:
	1) Go into the directory: "Platform/Linux/CreateRedist".
	   Run the script: "./RedistMaker".
	   This will compile everything and create a redist package in the "Platform/Linux/Redist" directory.
	   It will also create a distribution in the "Platform/Linux/CreateRedist/Final" directory.
	2) Go into the directory: "Platform/Linux/Redist".
	   Run the script: "sudo ./install.sh" (needs to run as root)
-----------------------------------------------
OpenCV with OpenNI -

1) cmake -DCMAKE_BUILD_TYPE=DEBUG -DBUILD_SHARED_LIBS=ON -DBUILD_PYTHON_SUPPORT=ON -DBUILD_ZLIB=ON -DBUILD_TIFF=ON -DBUILD_JASPER=ON -DBUILD_JPEG=ON -DBUILD_PNG=ON -DBUILD_OPENEXR=ON -DWITH_TBB=ON -DWITH_UNICAP=ON -DWITH_FFMPEG=ON -DWITH_OPENNI=ON ..
2) Check that it detects the OpenCV and PrimeSense Library correctly
3) make
4) sudo make install


