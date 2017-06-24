#include "opencv2/highgui/highgui.hpp"
#include "srrg_system_utils/system_utils.h"
#include "path_map.h"
#include "dijkstra_path_search.h"
#include "distance_map_path_search.h"
#include "path_map_utils.h"

#include <fstream>

using namespace std;

using namespace srrg_core;

Vector2iVector goals;
Eigen::Vector2i current_position;
FloatImage shown_image;
DijkstraPathSearch path_calculator;

// image with free and occupied space. Each occupied cell has unique index
// recomputed when touching the occupied and free thresholds
IntImage indices_image; 

// distance image. unknown cell have negative distance
// recomputed when touching the resolution and the safety_range, OR the parameters
// controlling the indices images are touched
FloatImage distance_image;

// cost_map. recomputed when max cost/ min cost or robot radius are touched or distance_image recomputed
FloatImage cost_image; // image of the costmap

// likelihood field, recomputed when parameters are changed
PathMap path_map; // path field

enum WhatToShow {Map, Distance, Cost};
WhatToShow what_to_show = Map;

float max_cost=100;
float min_cost=20;
float robot_radius=0.3;
float safety_region=1.0f;

float resolution=0.05;
int free_threshold=240;
int occ_threshold=60;

UnsignedCharImage gray_map;

// converts a map into an int image
// to each occuiped cell is assigned a unique value >=0
// to each free cell is assigned -1
// to each unknown cell is assigned -2;

bool recompute_occupancy=true;
bool recompute_distance=true;
bool recompute_cost=true;
bool recompute_obstacles=true;
bool paint_brush=false;
bool drawing=false;
Vector2iVector obstacle_points;
std::vector<unsigned char> previous_pixel_values;
static void mouseEventHandler( int event, int x, int y, int flags, void* userdata) {
  if (!paint_brush && event==cv::EVENT_LBUTTONDOWN){
    goals.push_back(Eigen::Vector2i(y,x));
    cerr << "Adding goal [" << goals.size() << " " << y << " " << x << "]" << endl;
  }
  if (!paint_brush && event==cv::EVENT_RBUTTONDOWN){
    current_position = Eigen::Vector2i(y,x);
    cerr << "Adding pose [ " << y << " " << x << "]" << endl;
  }
  if (paint_brush && event== cv::EVENT_LBUTTONDOWN){
    cerr << "Starting to draw" << endl;
    drawing = true;
    previous_pixel_values.push_back(gray_map.at<unsigned char>(y,x));
    gray_map.at<unsigned char>(y,x) = (unsigned char) 0;
    obstacle_points.push_back(Eigen::Vector2i(y,x));
  }
  if (paint_brush && event==cv::EVENT_LBUTTONUP){
    //This point is already included in MOUSEMOVE
    drawing = false;
    recompute_occupancy=true;
    recompute_distance=true;
    recompute_cost=true;
  }
  if (paint_brush && drawing && event==cv::EVENT_MOUSEMOVE){
    //cerr << "Drawing" << x << " " << y << endl;
    previous_pixel_values.push_back(gray_map.at<unsigned char>(y,x));
    gray_map.at<unsigned char>(y,x) = (unsigned char) 0;
    obstacle_points.push_back(Eigen::Vector2i(y,x));   
  }
}

void reset(){
  for (size_t i = 0; i<obstacle_points.size(); i++){
    Eigen::Vector2i p = obstacle_points[i];
    gray_map.at<unsigned char>(p.x(), p.y()) = (unsigned char) previous_pixel_values[i]; //TODO: previous value
  }
  recompute_occupancy=true;
  recompute_distance=true;
  recompute_cost=true; 

  obstacle_points.clear();
  previous_pixel_values.clear();
}

static void onOccupancyTrackbar(int, void*) {
  recompute_occupancy=true;
  recompute_distance=true;
  recompute_cost=true;
}

static void onDistanceTrackbar(int, void*) {
  recompute_distance=true;
  recompute_cost=true;
}

static void onCostTrackbar(int, void*) {
  recompute_cost=true;
}

void showStuff() {
  switch(what_to_show) {
  case Map:
    shown_image.resize(indices_image.rows, indices_image.cols);
    for (int r=0; r<indices_image.rows; ++r) {
      int* src_ptr=indices_image.ptr<int>(r);
      float* dest_ptr=shown_image.ptr<float>(r);
      for (int c=0; c<indices_image.cols; ++c, ++src_ptr, ++dest_ptr){
	if (*src_ptr<-1)
	  *dest_ptr = .5f;
	else if (*src_ptr == -1)
	  *dest_ptr = 1.f;
	else
	  *dest_ptr=0.f;
      }
    }
    break;
  case Distance:
    shown_image=distance_image*(1./safety_region);
    break;
    
  case Cost:
    shown_image=cost_image*(1.f/max_cost);
    break;

  }

  for (const Eigen::Vector2i& goal:goals) {
    cv::circle(shown_image, cv::Point(goal.y(), goal.x()), 3, cv::Scalar(100.0f)); 
  }
  cv::circle(shown_image, cv::Point(current_position.y(), current_position.x()), 3, cv::Scalar(100.0f), 3.0f);
  if (current_position.x()>=0 && goals.size()){
    std::ofstream ofs ("path.dat", std::ofstream::out);
    PathMapCell* current=&path_map(current_position.x(), current_position.y());
    while (current&& current->parent && current->parent!=current) {
      PathMapCell* parent=current->parent;
      cv::line(shown_image,
	       cv::Point(current->c, current->r),
	       cv::Point(parent->c, parent->r),
	       cv::Scalar(0.0f));
      ofs << current->r << " " << current->c << std::endl;

      current = current->parent;
    }
  }

  cv::imshow("path", shown_image);
}

int main(int argc, char** argv){
  
  cerr << "loading image from file: " << argv[1] << endl;
  cv::Mat grid_map= cv::imread(argv[1]);

  cvtColor(grid_map, gray_map, CV_BGR2GRAY);
  cvNamedWindow("controls");
  cv::createTrackbar("occ_threshold", "controls", &occ_threshold, 255, onOccupancyTrackbar);
  cv::createTrackbar("free_threshold", "controls", &free_threshold, 255, onOccupancyTrackbar);
  int resolution_in_mm=1000*resolution;
  cv::createTrackbar("resolution_in_mm", "controls", &resolution_in_mm, 500, onDistanceTrackbar);
  
  cvNamedWindow("path");
  int max_cost_int=max_cost;
  cv::createTrackbar("max_cost", "controls", &max_cost_int, 1000, onCostTrackbar);

  int min_cost_int=min_cost;
  cv::createTrackbar("min_cost", "controls", &min_cost_int, 1000, onCostTrackbar);

  int safety_region_in_mm=safety_region*1000;
  cv::createTrackbar("safety_region_in_mm", "controls", &safety_region_in_mm, 3000, onDistanceTrackbar);

  int robot_radius_in_mm=robot_radius*1000;
  cv::createTrackbar("robot_radius_in_mm", "controls", &robot_radius_in_mm, 3000, onCostTrackbar);

  
  cv::setMouseCallback("path", mouseEventHandler, NULL);

  shown_image.create(gray_map.rows, gray_map.cols);
  
  bool run=true;
  current_position.x()=-1;
  current_position.y()=-1;
  
  while(run) {

    if (recompute_occupancy) {
      grayMap2indices(indices_image, gray_map, occ_threshold, free_threshold);
      recompute_occupancy=false;
    }
    if (recompute_distance) {
      safety_region= 1e-3*safety_region_in_mm;
      resolution=1e-3*resolution_in_mm;
      indices2distances(distance_image, indices_image, resolution, safety_region);
      recompute_distance=false;
    }
    if (recompute_cost) {
      min_cost=min_cost_int;
      max_cost=max_cost_int;
      robot_radius=1e-3*robot_radius_in_mm;
      safety_region= 1e-3*safety_region_in_mm;
      distances2cost(cost_image,
		     distance_image,
		     robot_radius,
		     safety_region,
		     min_cost,
		     max_cost);
      recompute_cost=false;
    }
    
    path_calculator.setMaxCost(max_cost-1);
    path_calculator.setCostMap(cost_image);
    path_calculator.setOutputPathMap(path_map);
    if (! goals.empty() && current_position.x()>=0){
      double t_start=getTime();
      path_calculator.goals()=goals;
      path_calculator.compute();
      double t_end=getTime();
      cerr << "time: " << t_end - t_start << " " << path_calculator.numOperations() << endl;
    }
    showStuff();

    unsigned char key=cv::waitKey(10);
    switch(key){
    case 'x':
      goals.clear();
      current_position.x()=-1;
      current_position.y()=-1;
      break;
    case 'm':
      what_to_show=Map; break;
    case 'd':
      what_to_show=Distance; break;
    case 'c':
      what_to_show=Cost; break;
    case 'p': //Paint obstacles by dragging mouse
      paint_brush = !paint_brush;
      std::cerr << "Paint brush" << paint_brush << std::endl;
      break;
    case 'r': //Paint obstacles by dragging mouse
      reset();
      std::cerr << "Reset obstacles" << std::endl;
      break;
      
    case 27: run=false; break; //ESC key
    }
  }
}
