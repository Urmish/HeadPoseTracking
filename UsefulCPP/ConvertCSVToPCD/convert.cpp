#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>

using namespace std;
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  
  // Fill in the cloud data
  
  //std::ifstream file ( "Golden.csv" ); // declare file stream: http://www.cplusplus.com/reference/iostream/ifstream/
  //std::string value;
  FILE *fp;
  fp = fopen("Golden.csv", "r");
  int x1,y1,z1;
  vector<int> x;
  vector<int> y;
  vector<int> z;
  int max_x = 0;
  int max_y = 0;
  while (fscanf(fp, "%d,%d,%d\n", &x1, &y1,&z1) == 3)
  {
	printf("%d %d %d\n", x1,y1,z1);
	x.push_back(x1);
	y.push_back(y1);
	z.push_back(z1);
  }
  //while ( file.good() )
  //{
  //   getline ( file, value, ',' ); // read a string until next comma: http://www.cplusplus.com/reference/string/getline/
  //   std::cout << std::string( value, 1, value.length()-2 ); // display value removing the first and the last character from it
  //}
  cout <<"CSV Read, now adding\n";
  cloud.is_dense = false;
  cloud.points.resize (x.size());
  for (int i = 0; i < cloud.points.size(); ++i)
  {
    cloud.points[i].x = x[i];
    cloud.points[i].y = y[i];
    cloud.points[i].z = z[i];
  }
  
  cloud.width    = 1;
  cloud.height   = x.size();
  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
  return (0);
}
