#include "ros_planner.h"

using namespace std;
using namespace spqrel_navigation;

int main(int argc, char **argv){
    ros::init(argc, argv, "spqrel_planner_node");
    ros::NodeHandle n;

    //constructs the planner
    ROSPlanner* rosplanner = new ROSPlanner(n);

    rosplanner->getParams();

    //requests the map
    rosplanner->requestMap();
    
    // init the planner
    rosplanner->init();  

    ros::Rate r(10);
    while (ros::ok()){
      rosplanner->runOnce();
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}

