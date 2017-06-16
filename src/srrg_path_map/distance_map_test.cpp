#include "path_map.h"
#include "distance_map_path_search.h"
#include "srrg_system_utils/system_utils.h"

using namespace std;

using namespace srrg_core;

void drawPathMap(RGBImage& output_image, const FloatImage& distance, float d_max){
  output_image.create(distance.rows, distance.cols);

  for (int r=0; r<distance.rows; ++r){
    const float * dist_ptr = distance.ptr<const float>(r);
    cv::Vec3b * out_ptr = output_image.ptr<cv::Vec3b>(r);
    for (int c=0; c<distance.cols; ++c, ++out_ptr, ++dist_ptr) {
      const float& d=*dist_ptr;
      if (d<d_max){
	unsigned char gray_val=255-d;
	*out_ptr=cv::Vec3b(gray_val, gray_val, gray_val);
      } else {
	*out_ptr=cv::Vec3b(255,0,0);
      }
    }
  } 
}
		 
int main(int argc, char** argv){
  int rows=480;
  int cols=640;
  int num_points=100;

  // create an image containing the random points;
  IntImage points_image(rows, cols);
  points_image=-1;

  
  // generate n random points
  Vector2fVector points(num_points);
  for (size_t i=0; i<points.size(); i++){
    points[i]=Eigen::Vector2f(rows*drand48(), cols*drand48());
    int r=points[i].x();
    int c=points[i].y();
    points_image.at<int>(r,c)=i;
  }
  
  // build a distance map
  PathMap distance_map;
  RGBImage shown_image;
  

  float d_max=100;
  cv::namedWindow("distance map");
  
  int d_curr=0;

  DistanceMapPathSearch dmap_calculator;
  dmap_calculator.setOutputPathMap(distance_map);
  dmap_calculator.setIndicesImage(points_image);

  // show progressive construction of distance map
  while (d_curr<d_max) {
    dmap_calculator.setMaxDistance(d_curr);
    double t_start=getTime();
    dmap_calculator.init();
    dmap_calculator.compute();
    double t_end=getTime();
    
    drawPathMap(shown_image, dmap_calculator.distanceImage(), d_curr-1);
    cv::imshow("distance map", shown_image);
    cv::waitKey(10);
    cerr << "dist: " << d_curr << "  ops: " << dmap_calculator.numOperations()<<  " time: " << t_end-t_start << endl; 
    d_curr++;
  }



}
