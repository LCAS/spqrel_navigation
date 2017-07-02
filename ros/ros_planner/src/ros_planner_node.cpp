#include "ros_planner.h"

using namespace std;
using namespace thin_navigation;

int main(int argc, char **argv){
    ros::init(argc, argv, "ros_planner_node");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    //constructs the planner
    ROSPlanner* planner=new ROSPlanner(n);

    //_map_service_id
    std::string map_service_id;
    private_nh.param("map_service_id", map_service_id, std::string("static_map"));
    planner->setMapServiceId(map_service_id);
    cout << "ros_planner: [string] _map_service_id: " << map_service_id << endl;

    //requests the map
    planner->requestMap();

    //read parameters
    planner->setROSParams();
 
    // init the planner
    planner->init();  // this function sets a subscriber to laser topic 
                      // that is the main execution thread for the planner
    //planner->initGUI();

    //run baby run
    ros::spin();
    return 0;
}

