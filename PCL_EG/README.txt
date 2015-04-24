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



