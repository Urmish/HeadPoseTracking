#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <fstream>
#include <iostream>

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"
#include <time.h>
#include <sstream>

using namespace cv;
using namespace std;
static long int filenumber = 2; // Start from Capture2.csv

extern size_t Hack_iteration_count;
extern double Hack_t;

double time_sum = 0;
double match_ratio_sum = 0;
int iteration_count_sum = 0;
double fail = 0;

const float PI = 3.14159265358979323846264f;

bool closeEnough(const float& a, const float& b, const float& epsilon = std::numeric_limits<float>::epsilon()) {
    return (epsilon > std::abs(a - b));
}
float Angle[3];
void eulerAngles(float R[3][3]) {

    //check for gimbal lock
    if (closeEnough(R[0][2], -1.0f)) {
        float x = 0; //gimbal lock, value of x doesn't matter
        float y = PI / 2;
        float z = x + atan2(R[1][0], R[2][0]);
        Angle[0] =  x;
        Angle[1] = y; 
        Angle[2] = z;
      //  return Angle;
    } else if (closeEnough(R[0][2], 1.0f)) {
        float x = 0;
        float y = -PI / 2;
        float z = -x + atan2(-R[1][0], -R[2][0]);
        Angle[0] =  x;
        Angle[1] = y; 
        Angle[2] = z; 
      //  return Angle;
    } else { //two solutions exist
        float x1 = -asin(R[0][2]);
        float x2 = PI - x1;

        float y1 = atan2(R[1][2] / cos(x1), R[2][2] / cos(x1));
        float y2 = atan2(R[1][2] / cos(x2), R[2][2] / cos(x2));

        float z1 = atan2(R[0][1] / cos(x1), R[0][0] / cos(x1));
        float z2 = atan2(R[0][1] / cos(x2), R[0][0] / cos(x2));

        //choose one solution to return
        //for example the "shortest" rotation
        if ((std::abs(x1) + std::abs(y1) + std::abs(z1)) <= (std::abs(x2) + std::abs(y2) + std::abs(z2))) {
            Angle[0] =  x1;
            Angle[1] = y1; 
            Angle[2] = z1; 
         //   return Angle;
        } else {
            Angle[0] =  x2;
            Angle[1] = y2; 
            Angle[2] = z2; 
        //    return Angle;
         }
    }
}


void transformation (const char *config)
{
	bool isCSV = true;
	while(1)  
	{ // Execute until file not found 

		typedef PointMatcher<float> PM;
		typedef PM::DataPoints DP;
		typedef PM::Parameters Parameters;
		
		// Load point clouds
		//const DP ref(DP::load(argv[1]));
		//const DP data(DP::load(argv[2]));
		
		const DP ref(DP::load("dense/Golden.csv"));
		
		std::ostringstream fileNameStream("");
		fileNameStream << "dense/Capture" << filenumber  << ".csv";
		string Inputname = fileNameStream.str();
		filenumber++;
		
		ifstream myfile (Inputname.c_str());
		if( myfile.is_open() ) 
		{
			//cout <<endl <<endl <<endl <<"File "  <<Inputname<<" is now opened"<<endl;
			myfile.close();
		}
		else 
		{
			cout <<"Unable to open "  <<Inputname<<endl;
		// Printing average values
			//cout << "Config file is "<<config<<endl;
			//cout << "Average time for ICP to converge ="<< time_sum/(filenumber-2) <<endl;
			//cout << "Average iteration for ICP to converge ="<< iteration_count_sum/(filenumber-2) <<endl;
			//cout << "Average Match ratio for ICP ="<< match_ratio_sum/(filenumber-2) <<endl;
			//cout << "Total Failures ="<< fail <<endl;
			break;  // exit from whiledouble time_sum = 0;
		}
		
		const DP data(DP::load(Inputname.c_str()));
		//const DP data(DP::load("Final.csv"));
		// Create the default ICP algorithm
		PM::ICP icp;
		
		string configFile = config;
		
		if (configFile.empty())
		{
			// See the implementation of setDefault() to create a custom ICP algorithm
			break;
			cout << "Cannot open "<<config<<" config file"<<endl;
			icp.setDefault();
		}
		else
		{
			// load YAML config
			ifstream ifs(configFile.c_str());
			if (!ifs.good())
			{
				cout << "Cannot open config file" << configFile;
			}
			icp.loadFromYaml(ifs);
		}
		
		
		// Compute the transformation to express data in ref
		PM::TransformationParameters T;
		try
		{
			T = icp(data, ref);
		}
		catch(...)
		{
			fail++;
			continue;
		}
		
		
		// Transform data to express it in ref
		DP data_out(data);
		try
		{
			icp.transformations.apply(data_out, T);
		}
		catch(...)
		{
			fail++;
			continue;
		}
		
		// Safe files to see the results
		//cout << "Iteration Count: " << icp.iterationCount  << endl;
		//ref.save("test_ref.vtk");
		//data.save("test_data_in.vtk");
		//data_out.save("test_data_out.vtk");
		//cout << "Final transformation:" << endl << T << endl;
		
		// Updating transform matrix 
		// We don't have transformation matrix here
		//T(0,3) = offset_x;
		//T(1,3) = offset_y;
		//T(2,3) = offset_z;
		//cout << "Final transformation:" << endl << T << endl;
		
		//cout << "match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;
		//cout << "Iteration Count = " << Hack_iteration_count <<" Elapsed time : " << Hack_t <<endl;
		
		//Calculating Parameters
		 
		time_sum += Hack_t;
		match_ratio_sum += (double)icp.errorMinimizer->getWeightedPointUsedRatio();
		iteration_count_sum += (int)Hack_iteration_count;
		
		float R[3][3];
		
		for(int i = 0; i < 3; i++)
		{
		    for(int j = 0; j < 3; j++)
		        R[j][i] = T(i,j); // Euler angle function uses Column Major
		}
		
		eulerAngles(R); 
		
		    //cout << "Euler Angles : X  = "  << Angle[0] * 180/ PI  << ", Y = " << Angle[1] * 180/ PI << " , Z = " <<    Angle[2] * 180/ PI << endl;
		
		
		}
}

int main( int argc, char* argv[] )
{   
    	ofstream results ("result.txt");
	ifstream myfile (argv[1]);
	if( myfile.is_open() ) 
	{
		while(myfile.good())
		{
			string config_name;
			myfile >> config_name;
			cout << config_name <<endl;
			transformation(config_name.c_str());	
			results <<config_name<<","<<time_sum/(filenumber-fail-2)<<","<<iteration_count_sum/(filenumber-fail-2)<<","<<match_ratio_sum/(filenumber-fail-2)<<","<<fail<<endl;
			match_ratio_sum = 0;
			iteration_count_sum = 0;
			fail = 0;
			time_sum =0;
			filenumber = 2;

		}
	}
	else
	{
		cout <<"Error, could not open config file list\n"<<endl;
	}
	results.close();
	return 0;
}

