#include "dynamic_map.h"

namespace srrg_planner {
  
  DynamicMap::DynamicMap(){
    _map_resolution = 0.05;
    _map_inverse_resolution = 1./_map_inverse_resolution;
    _fov = 240*M_PI/180; //240 degrees for pepper
    _num_ranges = 60; // 60 for pepper
    _distance_threshold = 0.3;
    _time_threshold = std::numeric_limits<int>::max(); 
  }

  void DynamicMap::setCurrentPoints(const Vector2fVector& current_points){
    int curr_index = _occupied_cells.size();
    _current_points.clear();

    for (size_t i = 0; i < current_points.size(); i++){
      Eigen::Vector2f point = current_points[i];
      _current_points.insert(std::make_pair(curr_index, point));
      curr_index++;
    }
  }
  
  bool DynamicMap::findCellByIndex(const CellIndexMap& cellIndexMap, const int index, Eigen::Vector2i& cell){
    for (CellIndexMap::const_iterator it=cellIndexMap.begin(); it!=cellIndexMap.end(); it++){
      if (it->second == index){
	cell = it->first;
	return true;
      }
    }
    return false;
  }
  
  void DynamicMap::transformPointsToRobot(PointIndexMap& transformedPoints){
    //for each point in occupied cells transform in robot ref frame
    transformedPoints.clear();
    for (CellIndexMap::iterator it = _occupied_cells.begin(); it != _occupied_cells.end(); it++){
      int index = it->second;
      Eigen::Vector2i obstacle_image = it->first;
      Eigen::Vector2f obstacle_world = grid2world(obstacle_image);

      Eigen::Vector2f obstacle_map = _robot_pose_transform.inverse()*obstacle_world;
      transformedPoints.insert(std::make_pair(index, obstacle_map));
    }
  }
  
  void DynamicMap::transformPointsToMap(const PointIndexMap& points){
    //new map of cells
    CellIndexMap new_occupied_cells;
    for (PointIndexMap::const_iterator it = points.begin(); it!=points.end(); it++){
      int index = it->first;
      Eigen::Vector2f point = it->second;
      
      Eigen::Vector2i cell;
      if (!findCellByIndex(_occupied_cells, index, cell)){
	//New point
	Eigen::Vector2f obstacle_map = _robot_pose_transform*point;
	Eigen::Vector2i obstacle_grid = world2grid(obstacle_map);
	new_occupied_cells.insert(std::make_pair(obstacle_grid,index));
      }else{
	//otherwise we dont touch the point, just copy
	new_occupied_cells.insert(std::make_pair(cell,index));
      }
    }

    _occupied_cells = new_occupied_cells;
  }

  
  void DynamicMap::projectPoints(FloatVector& ranges, IntVector& indices, const PointIndexMap& points){

    ranges.resize(_num_ranges);
    indices.resize(_num_ranges);
    std::fill(ranges.begin(), ranges.end(), 1e3);
    std::fill(indices.begin(), indices.end(), -1);
    float middle = _num_ranges * 0.5;
    float inverse_angle_increment = 1. / _angle_increment;


    for (PointIndexMap::const_iterator it=points.begin(); it!=points.end(); it++){
      int index = it->first;
      Eigen::Vector2f point = it->second;
      
      float range = point.norm();
      float angle = atan2(point.y(), point.x());

      int bin = round(angle * inverse_angle_increment + middle);
      if (bin < 0 || bin >= _num_ranges)
	continue;
    
      float& brange = ranges[bin];
      if (brange > range){
	brange = range;
	indices[bin] = index;
      }else 
	continue;
    }
  }

  void DynamicMap::applyBlindZones(IntVector& indices){
    float middle = _num_ranges * 0.5;
    float inverse_angle_increment = 1. / _angle_increment;
    
    for (Eigen::Vector2f blind_zone: _blind_zones){
      float from, to;
      if (blind_zone.x() < blind_zone.y()){
	from = blind_zone.x();
	to = blind_zone.y();
      } else {
	from = blind_zone.y();
	to = blind_zone.x();
      }

      int bin_from = round(from * inverse_angle_increment + middle);
      int bin_to = round(to * inverse_angle_increment + middle);

      if (bin_from < 0 || bin_from > _num_ranges || bin_to < 0 || bin_to > _num_ranges){
	std::cerr << "Not valid in blind zone. From: " << bin_from << " to: " << bin_to << std::endl;
	continue;
      }
      // From [bin_from, bin_to), bin_to not included. bin_to<=num_ranges. 
      std::replace (indices.begin()+bin_from, indices.begin()+bin_to, -1, -2 );
    }


  }
  
  void DynamicMap::merge(PointIndexMap& points_merged,
			 const FloatVector& ranges_previous, const IntVector& indices_previous,
			 const PointIndexMap& points_previous,
			 const FloatVector& ranges_current, const IntVector& indices_current,
			 const PointIndexMap& points_current){

    points_merged = points_previous;
    
    std::vector<int> indices_to_remove;
    for (size_t i = 0; i < ranges_previous.size(); i++){
      int idx_prev = indices_previous[i];
      int idx_curr = indices_current[i];

      if (idx_prev < 0) {
	//There was no point previously in this bin,
	//We do not do anything
	continue;
      }

      // new point to the infinity, remove old one
      if (idx_curr == -1 ) {
	// Current point is in the infinity (-1)
	if (idx_prev > -1){
	  // If there was a point in this bin before we remove it
	  indices_to_remove.push_back(idx_prev);
	  idx_prev = -1;
	  continue;
	}
      } else if (idx_curr == -2 ) {
	// Current point is in blind zone,
	// We do not do anything
	continue;
      }

      float range_prev = ranges_previous[i];
      float range_curr = ranges_current[i];
      float distance = (range_prev - range_curr);

      if (distance < -_distance_threshold || fabs(distance) < _distance_threshold){
	//new point a lot behind or in the distance_threshold
	//remove old one
	indices_to_remove.push_back(idx_prev);
	idx_prev = -1;
	continue;
      }
    }

    // remove selected vertices
    for (int idx_to_remove: indices_to_remove)
      points_merged.erase(idx_to_remove);
    
    // add new points - current scan 
    points_merged.insert(points_current.begin(), points_current.end());

  }

  void DynamicMap::reindex(){
    int idx = 0;
    for (CellIndexMap::iterator it=_occupied_cells.begin(); it!=_occupied_cells.end(); it++, idx++){
      _occupied_cells[it->first] = idx;
    }
  }

  void DynamicMap::compute(){

    _angle_increment = _fov/_num_ranges;

    // Pixels of obstacles to robot frame
    PointIndexMap transformedPoints;
    transformPointsToRobot(transformedPoints);

    // Project to range and bearing bin
    // Old points
    FloatVector ranges_previous;
    IntVector indices_previous;
    projectPoints(ranges_previous, indices_previous, transformedPoints);

    // New points
    FloatVector ranges_current;
    IntVector indices_current;
    projectPoints(ranges_current, indices_current, _current_points);

    /*
    std::cerr << "Indices before: ";
    for (size_t i = 0; i<indices_current.size(); i++){
      std::cerr << indices_current[i] << " ";
    }
    std::cerr << std::endl;
    */
    
    // Consider blind zones (suitable for Pepper)
    applyBlindZones(indices_current);

    /*
    std::cerr << "Indices after: ";
    for (size_t i = 0; i<indices_current.size(); i++){
      std::cerr << indices_current[i] << " ";
    }
    std::cerr << std::endl;
    */
    
    PointIndexMap points_merged;
    merge(points_merged,
	  ranges_previous, indices_previous, transformedPoints,
	  ranges_current, indices_current, _current_points);

    transformPointsToMap(points_merged);

    updateTimes();
    
    reindex();
  }

  void DynamicMap::getOccupiedCells(Vector2iVector& occupied_cells){
    occupied_cells.resize(_occupied_cells.size());
    int i=0;
    for (CellIndexMap::iterator it=_occupied_cells.begin(); it!=_occupied_cells.end(); it++, i++){
     occupied_cells[i] = it->first;
    }
    
  }

  void DynamicMap::updateTimes(){
    //Must be used before reindex
    
    CellTimeMap new_time_cells;
    //First copy only remaining cells preserving old times
    for (CellIndexMap::iterator it = _occupied_cells.begin(); it!=_occupied_cells.end(); it++){
      Eigen::Vector2i cell = it->first;
      CellTimeMap::iterator it_time =_time_cells.find(cell);
      if (it_time != _time_cells.end()){
	new_time_cells.insert(*it_time);
      }//else should not happen
    }    

    //For current points set time to now()
    for (PointIndexMap::iterator it=_current_points.begin(); it!=_current_points.end(); it++) {
      int index = it->first;
      Eigen::Vector2i cell;
      if (findCellByIndex(_occupied_cells, index, cell)){
	CellTimeMap::iterator it =new_time_cells.find(cell); 
	if (it != new_time_cells.end()){
	  //Cell already in time_cells, update time
	  it->second = std::chrono::steady_clock::now();
	}else {
	  //Cell not in time_cells, insert
	  new_time_cells.insert(std::make_pair(cell, std::chrono::steady_clock::now()));
	}
      } // else we do not store time for a cell that is not in _occuppied_cells
    }

    _time_cells = new_time_cells;
        
    //Deleting cells older than time threshold
    std::chrono::steady_clock::time_point time_now =std::chrono::steady_clock::now();
    for (CellTimeMap::iterator it=new_time_cells.begin(); it!=new_time_cells.end(); it++){
      Eigen::Vector2i cell = it->first;
      std::chrono::steady_clock::time_point time_cell = it->second;

      int ellapsed_time = std::chrono::duration_cast<std::chrono::seconds>(time_now - time_cell).count();
      //std::cerr << "Cell: " << cell.transpose() << " ellapsed_time: " << ellapsed_time;

      if (ellapsed_time > _time_threshold){
	_occupied_cells.erase(cell);
	_time_cells.erase(cell);
	//std::cerr << " Removing";
      }
      //std::cerr << std::endl;
      
    }

    //std::cerr << "Size occ_cells: " << _occupied_cells.size() << " size time_cells: " << _time_cells.size() << std::endl; 

  }
  
  
  
  
  
}




