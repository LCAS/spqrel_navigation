#include "localization_filter.h"
#include <fstream>
#include <cstdlib>

using namespace srrg_localizer2d;
using namespace srrg_core;
using namespace std;

int main (int , char**) {
  TransitionModel transition_model;
  ParticleVector particles(1000);
  
  // go straight;
  std::ofstream os("particles.dat");
  for (int i = 0; i< 10; i++) {
    Eigen::Vector3f control(1,0,0);
    transition_model.prepareSampling(control);
    for (size_t j = 0; j< particles.size(); j++) {
      particles[j]._pose = transition_model.sample(particles[j]._pose);
      os << particles[j]._pose.transpose()<< endl;
    }
  }

  for (int i = 0; i< 10; i++) {
    Eigen::Vector3f control(0,0,M_PI/20);
    transition_model.prepareSampling(control);
    for (size_t j = 0; j< particles.size(); j++) {
      particles[j]._pose = transition_model.sample(particles[j]._pose);
      os << particles[j]._pose.transpose()<< endl;
    }
  }

  for (int i = 0; i< 10; i++) {
    Eigen::Vector3f control(1,0,0);
    transition_model.prepareSampling(control);
    for (size_t j = 0; j< particles.size(); j++) {
      particles[j]._pose = transition_model.sample(particles[j]._pose);
      os << particles[j]._pose.transpose()<< endl;
    }
  }

  for (int i = 0; i< 10; i++) {
    Eigen::Vector3f control(0,0,M_PI/20);
    transition_model.prepareSampling(control);
    for (size_t j = 0; j< particles.size(); j++) {
      particles[j]._pose = transition_model.sample(particles[j]._pose);
      os << particles[j]._pose.transpose()<< endl;
    }
  }

  for (int i = 0; i< 10; i++) {
    Eigen::Vector3f control(1,0,0);
    transition_model.prepareSampling(control);
    for (size_t j = 0; j< particles.size(); j++) {
      particles[j]._pose = transition_model.sample(particles[j]._pose);
      os << particles[j]._pose.transpose()<< endl;
    }
  }


  os.close();

  return 0;

}
