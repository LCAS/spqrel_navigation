#include <qi/applicationsession.hpp>

// Boost includes.
#include <boost/program_options.hpp>

#include "naoqi_planner_new.h"

using namespace spqrel_navigation;

namespace po = boost::program_options;

int main(int argc, char **argv){
  
  po::options_description description("Options");
  description.add_options()
    ("help", "Displays this help message")
    ("pip", po::value<std::string>()->default_value(std::getenv("PEPPER_IP")), "Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
    ("pport", po::value<int>()->default_value(9559), "Naoqi port number.")
    ("use_gui", po::value<bool>(), "use_gui.")
    ("map", po::value<std::string>(), "Map used for localization in YAML format with extension.")
    ("robot_radius", po::value<float>()->default_value(0.3), "robot_radius.")
    ("max_linear_vel", po::value<float>(), "max_linear_vel.")
    ("max_angular_vel", po::value<float>(), "max_angular_vel.")
    ("max_linear_acc", po::value<float>(), "max_linear_acc.")
    ("max_angular_acc", po::value<float>(), "max_angular_acc.")
    ("min_angular_vel", po::value<float>(), "min_angular_vel.")
    ("goal_translation_tolerance", po::value<float>(), "goal_translation_tolerance.")
    ("goal_rotation_tolerance", po::value<float>(), "goal_rotation_tolerance.")
    ("recovery_waiting_time", po::value<int>(), "recovery_waiting_time.")
    ("recovery_obstacle_distance", po::value<float>(), "recovery_obstacle_distance.")    
    //TODO: ("collision_protection_desired", po::value<bool>()->default_value(true), "Enable/Disable self collision obstacle avoidance.")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);
  
  // --help option
  if (vm.count("help")){ 
    std::cout << description << std::endl; 
    return 0; 
  }

  ////////////
  //NAOqi session initialization
  const std::string pip = vm["pip"].as<std::string>();
  int pport = vm["pport"].as<int>();
  
  std::string tcp_url("tcp://"+pip+":"+std::to_string(pport));

  qi::ApplicationSession app(argc, argv, 0, tcp_url);
  try {
    app.startSession();
  }
  catch (qi::FutureUserException e) {
    std::cerr << "Connection refused." << std::endl;
    exit(1);
  }

  std::cerr << "Connected to robot." << std::endl;

  qi::SessionPtr session = app.session();
  ////////////

  
  NAOqiPlanner* naoqiplanner = new NAOqiPlanner(session);

  naoqiplanner->getParams(vm);
  //setting planner parameters
  //TODO: planner->setExternalCollisionProtectionDesired(vm["collision_protection_desired"].as<bool>());

  // init the planner
  naoqiplanner->init();

  
  /* NEW: will be done in mapsubscriber
  //get map
  planner->readMap(mapname);
  */
  /* NEW: will be done in the init()
  bool use_gui = vm["use_gui"].as<bool>();
  if (use_gui)
    planner->initGUI();
  */


  naoqiplanner->start();
  app.run();
  naoqiplanner->stop();
  
}
