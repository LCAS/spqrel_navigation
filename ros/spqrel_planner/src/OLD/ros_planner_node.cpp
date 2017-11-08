#include "ros_planner.h"

using namespace std;
using namespace spqrel_navigation;

int main(int argc, char **argv){
    ros::init(argc, argv, "spqrel_planner_node");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    //constructs the planner
    ROSPlanner* rosplanner=new ROSPlanner(n);

    //_map_service_id
    std::string map_service_id;
    private_nh.param("map_service_id", map_service_id, std::string("static_map"));
    rosplanner->setMapServiceId(map_service_id);
    cout << "ros_planner: [string] _map_service_id: " << map_service_id << endl;

    //read parameters
    rosplanner->setROSParams();
 
    // init the planner
    rosplanner->init();  // this function sets a subscriber to laser topic 
                      // that is the main execution thread for the planner

    //requests the map
    //rosplanner->requestMap();

    //run baby run
    ros::spin();

    rosplanner->quit();

    return 0;
}

