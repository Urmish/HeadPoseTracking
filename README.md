Please Read the INSTALL_README and INFO_README before you proceed with this.

In each folder - 

1. Modify CMakeLists.txt to point to libraries in your machine
2. cmake .
3. make

###Training/
ICP Grid Search at GridSearch/

1. Matcher/Matcher_Normal - Show some of the configurations used
2. Results - CSV File of the corresponding results

###motion_predictor/
Matlab file for motion prediction training(main.m). 
Transformation_dump.csv is the file used for determining the motion predictor parameters

###modified_game_files/
Download Game from [here] (http://sauerbraten.org)

sauerbraten/src/engine/main.cpp was modified to integrate with our head tracker output. This folder contains the modified main.cpp file.

###CVNI_FaceDetect_ICP_Kush/
Complete program with game integration support.

./cvni

###UsefulCPP/GridSearch

C code used for grid search. Input, list of configuration files

###UsefulCPP/CVNI_FPS

This is an optimized code to get the FPS of our system


