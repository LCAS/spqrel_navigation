#include "naoqi_sensor_utils.h"

namespace naoqi_sensor_utils {
  using namespace srrg_core;

  /*
    static const std::Vector<Eigen::Isometry2f, Eigen::aligned_allocator<Eigen::Isometry2f> >
    _beam_stransforms;

    void initTransforms() {
    for  (int i=0; i<number_of_beams) {
    Eigen::Iseomtry3f
    _beams_transforms[i] = EigenL
    }
    }
  */
  
  // void rawPointsToRobotFrame(Vector2fVector& result, const Vector2fVector& rawPoints){
  //   result.resize(rawPoints.size());
  //   for (size_t i =0; i< rawPoints.size(); ++i ){
  //     result[i]=_beam_transforms[i]*rawPoints[i];
  //   }

  Vector2fVector rawPointsToRobotFrame(Vector2fVector& rawPoints, float max_range){
    Vector2fVector framePoints(rawPoints.size());
    size_t count = 0;
    for (size_t i=0; i<rawPoints.size(); i++){
      Eigen::Vector2f rawPoint = rawPoints[i];
      if (i<15){
	//Right Laser point
	float lX = -0.01800;
	float lY = -0.08990;
	float lt = -1.57079;
	float tX = rawPoint[0]*cos(lt) - rawPoint[1]*sin(lt) + lX;
	float tY = rawPoint[0]*sin(lt) + rawPoint[1]*cos(lt) + lY;
	Eigen::Vector2f transformedPoint(tX,tY);
	if (transformedPoint.norm() < max_range)
	  framePoints[count++] = transformedPoint;
	continue;
      }
      if (i<30){
	//Front Laser point
	float lX = 0.05620;
	float lY = 0.0;
	float lt = 0.0;
	float tX = rawPoint[0]*cos(lt) - rawPoint[1]*sin(lt) + lX;
	float tY = rawPoint[0]*sin(lt) + rawPoint[1]*cos(lt) + lY;
	Eigen::Vector2f transformedPoint(tX,tY);
	if (transformedPoint.norm() < max_range)
	  framePoints[count++] = transformedPoint;
	continue;
      }
      if (i<45){
	//Leftt Laser point
	float lX = -0.01800;
	float lY =  0.08990;
	float lt =  1.57079;
	float tX = rawPoint[0]*cos(lt) - rawPoint[1]*sin(lt) + lX;
	float tY = rawPoint[0]*sin(lt) + rawPoint[1]*cos(lt) + lY;
	Eigen::Vector2f transformedPoint(tX,tY);
	if (transformedPoint.norm() < max_range)
	  framePoints[count++] = transformedPoint;
	continue;
      }
    }
    framePoints.resize(count);
    return framePoints;
  }

  Vector2fVector getLaser(qi::AnyObject memory_service, float max_range){
    std::vector<std::string> laserKeysVector(laserMemoryKeys, std::end(laserMemoryKeys)); 
    qi::AnyValue readings = memory_service.call<qi::AnyValue>("getListData", laserKeysVector);

    std::vector<float> floatReadings;
    try {
        floatReadings = readings.toList<float>();
    }
    catch (std::runtime_error e) {
        // std::cerr << "runtime error catched!!!" << std::endl;
        Vector2fVector points;
        return points;
    }

    //Preparing XY pairs
    Vector2fVector points(floatReadings.size()/2);
    for (size_t i = 0; i < floatReadings.size()-1; ){
      Eigen::Vector2f p(floatReadings[i], floatReadings[i+1]);
      points[i/2]=p;
      i=i+2;
    }
    Vector2fVector endpoints = rawPointsToRobotFrame(points, max_range);
    return endpoints;
  }

}
