#include <qi/applicationsession.hpp>

// Boost includes.
#include <boost/program_options.hpp>

#include "naoqi_navigation_gui.h"

using namespace naoqi_navigation_gui;

namespace po = boost::program_options;

int main(int argc, char **argv){
  
  po::options_description description("Options");
  description.add_options()
    ("help", "Displays this help message")
    ("pip", po::value<std::string>()->default_value(std::getenv("PEPPER_IP")), "Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
    ("pport", po::value<int>()->default_value(9559), "Naoqi port number.")
    ("map", po::value<std::string>(), "Map used for localization in YAML format with extension.")
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

  NAOqiNavigationGUI* navigation_gui = new NAOqiNavigationGUI(session);

  //get map
  navigation_gui->readMap(mapname);

  navigation_gui->initGUI();
  
  navigation_gui->subscribeServices();
  app.run();
  navigation_gui->unsubscribeServices();
  
}
