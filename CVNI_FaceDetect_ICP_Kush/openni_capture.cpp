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


using namespace cv;
using namespace std;
static int filenumber = 1;
void detectAndDisplay( Mat rgbframe, Mat depthframe , int argc, char *argv[]);

/** Global variables */
//-- Note, either copy these two files from opencv/data/haarscascades to your current folder, or change these locations
const char *cascade_name[2]={"haar/haarcascade_frontalface_default.xml",
			     "haar/nose.xml",
			    };
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "Capture - Face detection";
RNG rng(12345);

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
        cout <<" Case 1";
      //  return Angle;
    } else if (closeEnough(R[0][2], 1.0f)) {
        float x = 0;
        float y = -PI / 2;
        float z = -x + atan2(-R[1][0], -R[2][0]);
        Angle[0] =  x;
        Angle[1] = y; 
        Angle[2] = z; 
        cout <<" Case 2";
      //  return Angle;
    } else { //two solutions exist
        float x1 = -asin(R[0][2]);
        float x2 = PI - x1;

        float y1 = atan2(R[1][2] / cos(x1), R[2][2] / cos(x1));
        float y2 = atan2(R[1][2] / cos(x2), R[2][2] / cos(x2));

        float z1 = atan2(R[0][1] / cos(x1), R[0][0] / cos(x1));
        float z2 = atan2(R[0][1] / cos(x2), R[0][0] / cos(x2));

        cout <<" Case 3";
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



void transformation (int argc, char *argv[])
{
	bool isCSV = true;
	
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::Parameters Parameters;
	
	// Load point clouds
	//const DP ref(DP::load(argv[1]));
	//const DP data(DP::load(argv[2]));

	const DP ref(DP::load("Golden.csv"));
	const DP data(DP::load("Final.csv"));
	// Create the default ICP algorithm
	PM::ICP icp;
	
    string configFile = "my_config.yaml";
    
	if (configFile.empty())
	{
		// See the implementation of setDefault() to create a custom ICP algorithm
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
	PM::TransformationParameters T = icp(data, ref);
	

	// Transform data to express it in ref
	DP data_out(data);
	icp.transformations.apply(data_out, T);
	
	// Safe files to see the results
	ref.save("test_ref.vtk");
	data.save("test_data_in.vtk");
	data_out.save("test_data_out.vtk");
	cout << "Final transformation:" << endl << T << endl;

    float R[3][3];
    
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
            R[j][i] = T(i,j); // Euler angle function uses Column Major
    }

    eulerAngles(R); 

	cout << "Euler Angles : X  = "  << Angle[0] * 180/ PI  << ", Y = " << Angle[1] * 180/ PI << " , Z = " <<    Angle[2] * 180/ PI << endl;

}

void matToCSV (Mat& image )
{
	Mat temp;
	image.convertTo(temp, CV_32S);
	string name;
	if (filenumber == 1)
	{
		name = "Golden.csv";
        filenumber++;
	}
	else
	{
		name = "Final.csv";
	}
	//ofstream Face_Template ("Face_Template.csv"); 
	ofstream Face_Template (name.c_str()); 
	int rows = image.rows;
	int cols = image.cols;

    // Finding Depth of nose, assuming its minimum
    int nose_depth = 2147483647;
    int FACE_DEPTH = 50; // 50 mm inwards into the face
	for (int i=0;i<rows;i++)
	{
		for (int j=0;j<cols;j++)
		{
			int depth = temp.at<int>(i,j);
			if (depth != 0 && depth < nose_depth)
			{
                nose_depth = depth;
			}
		}
	}


	for (int i=0;i<rows;i++)
	{
		for (int j=0;j<cols;j++)
		{
			int depth = temp.at<int>(i,j);
			if (depth != 0 && depth < (nose_depth + FACE_DEPTH))
			{
			      Face_Template << i <<","<<j <<"," <<depth << endl;
			}
		}
	}
	Face_Template.close();
}

static void help()
{
        cout << "\nThis program demonstrates usage of depth sensors (Kinect, XtionPRO,...).\n"
                        "The user gets some of the supported output images.\n"
            "\nAll supported output map types:\n"
            "1.) Data given from depth generator\n"
            "   OPENNI_DEPTH_MAP            - depth values in mm (CV_16UC1)\n"
            "   OPENNI_POINT_CLOUD_MAP      - XYZ in meters (CV_32FC3)\n"
            "   OPENNI_DISPARITY_MAP        - disparity in pixels (CV_8UC1)\n"
            "   OPENNI_DISPARITY_MAP_32F    - disparity in pixels (CV_32FC1)\n"
            "   OPENNI_VALID_DEPTH_MASK     - mask of valid pixels (not ocluded, not shaded etc.) (CV_8UC1)\n"
            "2.) Data given from RGB image generator\n"
            "   OPENNI_BGR_IMAGE            - color image (CV_8UC3)\n"
            "   OPENNI_GRAY_IMAGE           - gray image (CV_8UC1)\n"
         << endl;
}


static void printCommandLineParams()
{
    cout << "-cd       Colorized disparity? (0 or 1; 1 by default) Ignored if disparity map is not selected to show." << endl;
    cout << "-fmd      Fixed max disparity? (0 or 1; 0 by default) Ignored if disparity map is not colorized (-cd 0)." << endl;
    cout << "-mode     image mode: resolution and fps, supported three values:  0 - CV_CAP_OPENNI_VGA_30HZ, 1 - CV_CAP_OPENNI_SXGA_15HZ," << endl;
    cout << "          2 - CV_CAP_OPENNI_SXGA_30HZ (0 by default). Ignored if rgb image or gray image are not selected to show." << endl;
    cout << "-m        Mask to set which output images are need. It is a string of size 5. Each element of this is '0' or '1' and" << endl;
    cout << "          determine: is depth map, disparity map, valid pixels mask, rgb image, gray image need or not (correspondently)?" << endl ;
    cout << "          By default -m 01010 i.e. disparity map and rgb image will be shown." << endl ;
    cout << "-r        Filename of .oni video file. The data will grabbed from it." << endl ;
}

static void parseCommandLine( int argc, char* argv[], bool& isColorizeDisp, bool& isFixedMaxDisp, int& imageMode, bool retrievedImageFlags[],
                       string& filename, bool& isFileReading )
{
    // set defaut values
    isColorizeDisp = true;
    isFixedMaxDisp = false;
    imageMode = 0;

    retrievedImageFlags[0] = false;
    retrievedImageFlags[1] = true;
    retrievedImageFlags[2] = false;
    retrievedImageFlags[3] = true;
    retrievedImageFlags[4] = false;

    filename.clear();
    isFileReading = false;

    if( argc == 1 )
    {
        help();
    }
    else
    {
        for( int i = 1; i < argc; i++ )
        {
            if( !strcmp( argv[i], "--help" ) || !strcmp( argv[i], "-h" ) )
            {
                printCommandLineParams();
                exit(0);
            }
            else if( !strcmp( argv[i], "-cd" ) )
            {
                isColorizeDisp = atoi(argv[++i]) == 0 ? false : true;
            }
            else if( !strcmp( argv[i], "-fmd" ) )
            {
                isFixedMaxDisp = atoi(argv[++i]) == 0 ? false : true;
            }
            else if( !strcmp( argv[i], "-mode" ) )
            {
                imageMode = atoi(argv[++i]);
            }
            else if( !strcmp( argv[i], "-m" ) )
            {
                string mask( argv[++i] );
                if( mask.size() != 5)
                    CV_Error( CV_StsBadArg, "Incorrect length of -m argument string" );
                int val = atoi(mask.c_str());

                int l = 100000, r = 10000, sum = 0;
                for( int j = 0; j < 5; j++ )
                {
                    retrievedImageFlags[j] = ((val % l) / r ) == 0 ? false : true;
                    l /= 10; r /= 10;
                    if( retrievedImageFlags[j] ) sum++;
                }

                if( sum == 0 )
                {
                    cout << "No one output image is selected." << endl;
                    exit(0);
                }
            }
            else if( !strcmp( argv[i], "-r" ) )
            {
                filename = argv[++i];
                isFileReading = true;
            }
            else
            {
                cout << "Unsupported command line argument: " << argv[i] << "." << endl;
                exit(-1);
            }
        }
    }
}

/*
 * To work with Kinect or XtionPRO the user must install OpenNI library and PrimeSensorModule for OpenNI and
 * configure OpenCV with WITH_OPENNI flag is ON (using CMake).
 */
int main( int argc, char* argv[] )
{
    bool isColorizeDisp, isFixedMaxDisp;
    int imageMode;
    bool retrievedImageFlags[5];
    string filename;
    bool isVideoReading;
    //parseCommandLine( argc, argv, isColorizeDisp, isFixedMaxDisp, imageMode, retrievedImageFlags, filename, isVideoReading );

    cout << "Device opening ..." << endl;
    cout << CV_CAP_OPENNI <<endl;
    VideoCapture capture;
    if( isVideoReading )
        capture.open( filename );
    else
        capture.open(CV_CAP_OPENNI);

    cout << "done." << endl;

    if( !capture.isOpened() )
    {
        cout << "Can not open a capture object." << endl;
        return -1;
    }

    if( !isVideoReading )
    {
        bool modeRes=false;
        switch ( imageMode )
        {
            case 0:
                modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
                break;
            case 1:
                modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_SXGA_15HZ );
                break;
            case 2:
                modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_SXGA_30HZ );
                break;
                //The following modes are only supported by the Xtion Pro Live
            case 3:
                modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_QVGA_30HZ );
                break;
            case 4:
                modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_QVGA_60HZ );
                break;
            default:
                CV_Error( CV_StsBadArg, "Unsupported image mode property.\n");
        }
        if (!modeRes)
            cout << "\nThis image mode is not supported by the device, the default value (CV_CAP_OPENNI_SXGA_15HZ) will be used.\n" << endl;
    }
    if(capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ) == 0) capture.set(CV_CAP_PROP_OPENNI_REGISTRATION,1);
    // Print some avalible device settings.
    cout << "\nDepth generator output mode:" << endl <<
            "FRAME_WIDTH      " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
            "FRAME_HEIGHT     " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
            "FRAME_MAX_DEPTH  " << capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
            "FPS              " << capture.get( CV_CAP_PROP_FPS ) << endl <<
            "REGISTRATION     " << capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ) << endl;
    if( capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) )
    {
        cout <<
            "\nImage generator output mode:" << endl <<
            "FRAME_WIDTH   " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) << endl <<
            "FRAME_HEIGHT  " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
            "FPS           " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) << endl;
    }
    else
    {
        cout << "\nDevice doesn't contain image generator." << endl;
        if (!retrievedImageFlags[0] && !retrievedImageFlags[1] && !retrievedImageFlags[2])
            return 0;
    }
    if( !face_cascade.load( cascade_name[0] ) )
    { 
	printf("--(!)Error loading\n"); return -1; 
    };
    if( !eyes_cascade.load( cascade_name[1] ) )
    { 
	printf("--(!)Error loading\n"); return -1; 
    };
    //printf("Entering for\n");

    for(;;)
    {
        Mat depthMap;
        Mat validDepthMap;
        Mat disparityMap;
        Mat bgrImage;
        Mat grayImage;
        Mat show; 

        if( !capture.grab() )
        {
            cout << "Can not grab images." << endl;
            return -1;
        }
        else
        {
            if( capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP ) )
            {
                const float scaleFactor = 0.05f;
		depthMap.convertTo( show, CV_8UC1, scaleFactor );
                imshow( "depth map", show );
            }

            if( capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE ) )
                imshow( "rgb image", bgrImage );

        /*    if( retrievedImageFlags[4] && capture.retrieve( grayImage, CV_CAP_OPENNI_GRAY_IMAGE ) )
                imshow( "gray image", grayImage );*/
	    if(!depthMap.empty() && !bgrImage.empty())
		    detectAndDisplay(bgrImage, depthMap, argc, argv);
	    
	    //writeMatToFile("depth.txt",depthMap);
        }

        if( waitKey( 30 ) >= 0 )
            break;
    }

    return 0;
}


void detectAndDisplay( Mat rgbframe, Mat depthframe, int argc, char *argv[] )
{
    //cv::FileStorage file("face_template.xml", cv::FileStorage::WRITE);
    //printf("detectAndDisplay\n");	
    Mat frame_gray;
    std::vector<Rect> faces;

   cvtColor( rgbframe, frame_gray, COLOR_BGR2GRAY );
   equalizeHist( frame_gray, frame_gray );
   //-- Detect faces
   //VIOLA JONESSSSSSSSSSSSSSS
   face_cascade.detectMultiScale( frame_gray, faces, 1.1, 4, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
   //face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

   double min_g = DBL_MAX;  
   cv::Point min_loc_g;
   int face_index = 0;
   int found = 0; 
   Rect nose;
   Point nose_center;
   //printf("detectAndDisplay faces found %d\n",faces.size());	
   for( size_t i = 0; i < faces.size(); i++ )
    {

      Mat faceROI = frame_gray( faces[i] );
      std::vector<Rect> eyes;
      //printf("min is %f\n",min);
            //-- In each face, detect eyes
      //eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );
      eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

      for( size_t j = 0; j < eyes.size(); j++ )
       {
         //Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
         //circle( rgbframe, eye_center, radius, Scalar( 255, 0, 0 ), 3, 8, 0 );
      	 Mat d_rect = faceROI(eyes[j]);
      	 double min,max;
      	 cv::Point min_loc,max_loc;
      	 cv::minMaxLoc(d_rect, &min, &max, &min_loc, &max_loc);
	 if (min < min_g)
         {
	 	min_g = min;
		face_index = i;
		min_loc_g.x = faces[i].x + min_loc.x ;
		min_loc_g.y = faces[i].y + min_loc.y ;
		nose = eyes[j];
                nose_center.x = faces[i].x + eyes[j].x + eyes[j].width/2;
		nose_center.y = faces[i].y + eyes[j].y + eyes[j].height/2 ;
		
         }      
      	 found = 1;

       }
    }
   //-- Show what you got
   if (found == 1)
   {
   	Point center( faces[face_index].x + faces[face_index].width/2, faces[face_index].y + faces[face_index].height/2 );
	ellipse( rgbframe, center, Size( faces[face_index].width/2, faces[face_index].height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 2, 8, 0 );
   	printf("Depth is %f\n",min_g);
        int radius = cvRound( (nose.width + nose.height)*0.25 );
        circle( rgbframe, nose_center, radius, Scalar( 255, 0, 0 ), 3, 8, 0 );
   	imshow( window_name, rgbframe );
      	Mat d_rect = depthframe(faces[face_index]);
       	//imshow("Face Depth",depthframe);  
        //imshow("Rectangle Depth",d_rect);
	printf("Nose depth is nose_center %d \n",depthframe.at<int>(nose_center.x,nose_center.y));
	//file<<"depthMap"<<d_rect(nose);
	//file<<"depthMap"<<d_rect;
	//int filenumber2 = 1;
	//cin >> filenumber2;
	printf("Point is %d,%d \n",min_loc_g.x,min_loc_g.y);
	if (filenumber == 1 )
	{
		matToCSV(d_rect);
	}
	else
	{
		matToCSV(d_rect);
		transformation(argc, argv);
	}
	
   }
}

