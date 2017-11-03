#include "ros_planner_new.h"

using namespace std;
using namespace spqrel_navigation;

int main(int argc, char **argv){
    ros::init(argc, argv, "spqrel_planner_node");
    ros::NodeHandle n;
    //ros::NodeHandle private_nh("~");

    //constructs the planner
    ROSPlanner* rosplanner = new ROSPlanner(n);

    rosplanner->getParams();

    //requests the map
    rosplanner->requestMap();
    
    // init the planner
    rosplanner->init();  // this function sets a subscriber to laser topic 
                         // that is the main execution thread for the planner

    ros::Rate r(10);
    while (ros::ok()){
      rosplanner->run();
      ros::spinOnce();
      r.sleep();
    }
    //rosplanner->quit();

    return 0;
}

