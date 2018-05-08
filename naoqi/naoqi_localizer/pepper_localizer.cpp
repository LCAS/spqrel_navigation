#include <qi/applicationsession.hpp>

// Boost includes.
#include <boost/program_options.hpp>

#include <string>
#include "naoqi_localizer.h"

using namespace naoqi_localizer;

namespace po = boost::program_options;

void startLocalizerFromParameters(NAOqiLocalizer* localizer, po::variables_map& vm){

  std::cerr << "Localizer Parameters: " << std::endl;
  int particles = vm["particles"].as<int>();
  std::cerr << "particles: " << particles << std::endl;
  double max_range = vm["max_range"].as<double>();
  std::cerr << "max_range: " << max_range << std::endl;
  double min_weight = vm["min_weight"].as<double>();
  std::cerr << "min_weight: " << min_weight << std::endl;
  double distance_threshold = vm["distance_threshold"].as<double>();
  std::cerr << "distance_threshold: " << distance_threshold << std::endl;
  bool dynamic_restart = vm["dynamic_restart"].as<bool>();
  std::cerr << "dynamic_restart: " << dynamic_restart << std::endl;
  bool use_gui = vm["use_gui"].as<bool>();
  std::cerr << "use_gui: " << use_gui << std::endl;
  bool use_d2l = vm["use_d2l"].as<bool>();
  std::cerr << "use_d2l: " << use_d2l << std::endl;

  double initial_pose_x = vm["initial_pose_x"].as<double>();
  std::cerr << "initial_pose_x: " << initial_pose_x << std::endl;
  double initial_pose_y = vm["initial_pose_y"].as<double>();
  std::cerr << "initial_pose_y: " << initial_pose_y << std::endl;
  double initial_pose_theta = vm["initial_pose_theta"].as<double>();
  std::cerr << "initial_pose_theta: " << initial_pose_theta << std::endl;

  localizer->init(particles, distance_threshold, 0.2, min_weight);
  if (use_gui)
    localizer->initGUI();
  localizer->setUseD2L(use_d2l);
  localizer->setParticleResetting(dynamic_restart);
  localizer->setInitialPose(initial_pose_x,initial_pose_y,initial_pose_theta);
}


int main(int argc, char **argv)
{
  std::string pepper_ip = "";
  if (std::getenv("PEPPER_IP") != NULL)
    pepper_ip = std::getenv("PEPPER_IP");
  
  po::options_description description("Options");
  description.add_options()
    ("help", "Displays this help message")
    ("pip", po::value<std::string>()->default_value(pepper_ip), "Robot IP address. Set IP here or for convenience, define PEPPER_IP as environment variable. On robot or Local Naoqi: use '127.0.0.1'.")
    ("pport", po::value<int>()->default_value(9559), "Naoqi port number.")
    ("map", po::value<std::string>(), "Map used for localization in YAML format with extension.")
    ("particles", po::value<int>()->default_value(1000), "particles.")
    ("max_range", po::value<double>()->default_value(10.0), "max_range.")
    ("min_weight", po::value<double>()->default_value(200.0), "min_weight.")
    ("distance_threshold", po::value<double>()->default_value(1.0), "distance_threshold.")
    ("dynamic_restart", po::value<bool>()->default_value(false), "dynamic_restart.")
    ("use_gui", po::value<bool>()->default_value(false), "use_gui.")
    ("use_d2l", po::value<bool>()->default_value(false), "use depth2laser data.")
    ("initial_pose_x", po::value<double>()->default_value(0.0), "initial_pose_x.")
    ("initial_pose_y", po::value<double>()->default_value(0.0), "initial_pose_y.")
    ("initial_pose_theta", po::value<double>()->default_value(0.0), "initial_pose_theta.")
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

  if (pip == ""){
    std::cerr << "PEPPER_IP not defined. Please, set robot ip through program options" << std::endl;
    exit(0);
  }
  
  std::string tcp_url("tcp://"+pip+":"+std::to_string(pport));

  qi::ApplicationSession app(argc, argv, 0, tcp_url);
  try {
    app.startSession();
  }
  catch (qi::FutureUserException e) {
    std::cerr << "Connection refused." << std::endl;
    exit(1);
  }


  qi::SessionPtr session = app.session();
 
  NAOqiLocalizer* localizer = new NAOqiLocalizer(session);
  localizer->readMap(mapname);
  startLocalizerFromParameters(localizer, vm);

  localizer->subscribeServices();
  app.run();
  localizer->unsubscribeServices();
}
