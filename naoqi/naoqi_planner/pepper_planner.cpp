#include <qi/applicationsession.hpp>

// Boost includes.
#include <boost/program_options.hpp>

#include "naoqi_planner.h"

using namespace naoqi_planner;

namespace po = boost::program_options;

int main(int argc, char **argv){
  
  po::options_description description("Options");
  description.add_options()
    ("help", "Displays this help message")
    ("pip", po::value<std::string>()->default_value(std::getenv("PEPPER_IP")), "Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
    ("pport", po::value<int>()->default_value(9559), "Naoqi port number.")
    ("map", po::value<std::string>(), "Map used for localization in YAML format with extension.")
    ("max_cost", po::value<float>()->default_value(100.0), "max_cost.")
    ("min_cost", po::value<float>()->default_value(20.0), "min_cost.")
    ("robot_radius", po::value<float>()->default_value(0.3), "robot_radius.")
    ("safety_region", po::value<float>()->default_value(1.0), "safety_region.")
    ("use_gui", po::value<bool>()->default_value(false), "use_gui.")
    ("collision_protection_desired", po::value<bool>()->default_value(true), "Enable/Disable self collision obstacle avoidance.")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);
  
  // --help option
  if (vm.count("help")){ 
    std::cout << description << std::endl; 
    return 0; 
  }

  // --map option
  std::string mapname;
  if (!vm.count("map")){ 
    std::cout << "No map provided. Exiting." << std::endl; 
    return 0; 
  }else {
    mapname =  vm["map"].as<std::string>();
  }
  
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

  NAOqiPlanner* planner = new NAOqiPlanner(session);

  //setting planner parameters
  planner->setMaxCost(vm["max_cost"].as<float>());
  planner->setMinCost(vm["min_cost"].as<float>());
  planner->setRobotRadius(vm["robot_radius"].as<float>());
  planner->setSafetyRegion(vm["safety_region"].as<float>());
  planner->setExternalCollisionProtectionDesired(vm["collision_protection_desired"].as<bool>());
  
  //get map
  planner->readMap(mapname);

  bool use_gui = vm["use_gui"].as<bool>();
  if (use_gui)
    planner->initGUI();
  
  planner->subscribeServices();
  app.run();
  planner->unsubscribeServices();
  
}
