#pragma once
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <voxblox_map/voxblox_map.h>
#include <string>
#include <planning/modules/SensorModel.h>
#include "ros/ros.h"
// #include <planning/modules/utils.h>



struct FrontierPoint
{
  Eigen::Vector3d frontier ; 
  Eigen::Vector3d occ_point ; 
};


class ViewEvaluator
{
private:
    /* data */
    std::string m_method_type;
    voxblox_map::VoxbloxMap m_map;
    SensorModel sensor_model_ ; 
    bool m_occlusion ; 
    BoundingBox m_bb;

    double m_angle_high;
    double m_angle_low; 

    float m_dist_min;
    float m_dist_max;

    // params
    double p_ray_step_;
    double p_downsampling_factor_;  // Artificially reduce the minimum resolution
    // to increase performance

    // constants
    int c_res_x_;  // factual resolution that is used for ray casting
    int c_res_y_;
    int c_n_sections_;  // number of ray duplications
    std::vector<double>
        c_split_distances_;            // distances where rays are duplicated
    std::vector<int> c_split_widths_;  // number of max distance rays that are
                                      // covered per split

    // variables
    Eigen::ArrayXXi ray_table_;

    //parameters for the view evaluation
    float value_frontier_ ; 
    double m_threshold_known ;
    int distance_surface_radius_max_ ; 
    //NEIGHBOURS VOXELS
    Eigen::Vector3d c_neighbor_voxels_[26];
    bool m_frontier6;

    std::vector<QualityArea> m_qa ; 

    bool verbose_;

public:
    ViewEvaluator(
      const voxblox_map::VoxbloxMap& map,
      const std::string& method_name,
      const SensorModel& sensor_lidar,
      double threshold,
      int radius_max_surface,
      float dist_min,
      float dist_max,
      bool occlusion,
      float angle_low,
      float angle_high,
      const BoundingBox& bb,
      const std::vector<QualityArea>& qa, 
      bool verbose
    );
    ViewEvaluator(){};
    ~ViewEvaluator();

    // float evaluate_view_pos(const ViewCandidate& vc, const ViewCandidate& current_pos);

    bool isSurfaceFrontier_TSDF(const Eigen::Vector3d& voxel, Eigen::Vector3d& unknown_vox, unsigned char& current_state);
    bool isLowConfidenceVoxel(const Eigen::Vector3d& voxel);


    bool lineOfSightCheck(const ViewCandidate& vc,  float& nb_unknown_voxels );
    float inverseRayCast(const ViewCandidate& vc, const std::vector<Eigen::Vector3d>& frontiers_set, int& nb_frontiers_visible, std::unordered_map<Eigen::Vector3d, int, Vector3dHash, Vector3dEqual>& visible_frontiers, bool save_reached_frontiers);
    float inverseRayCast_no_distance(const ViewCandidate& vc, const std::vector<Eigen::Vector3d>& frontiers_set, int& nb_frontiers_visible, std::unordered_map<Eigen::Vector3d, int, Vector3dHash, Vector3dEqual>& visible_frontiers, bool save_reached_frontiers);
    float inverseRayCast_simple(const ViewCandidate& vc,const std::vector<Eigen::Vector3d>& to_observe_set, int& nb_observed);
    float ponder_frontier_by_distance(float frontier_dist_to_vc);
    float ponder_frontier_by_distance_multi_resolution(float frontier_dist_to_vc, float dist_min, float dist_max);



    float evaluate_view_angular(const ViewCandidate& vc, const Eigen::Vector3d& robot_pos, const Eigen::Vector3d& vel);
    float evaluate_distance_angle_cost( float value_angular, float dist_min_frontiers, float dist_view );
    float getMinMaxViewDistance(std::vector<ViewCandidate> views, const Eigen::Vector3d& current_pos_vector,  float& max_dist);

    void getVisibleVoxels_camera(const ViewCandidate& vc);
    void getVisibleVoxels_LIDAR(std::vector<Eigen::Vector3d>* result, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
    void markNeighboringRays(int x, int y, int segment, int value);

};

ViewEvaluator::ViewEvaluator(
  const voxblox_map::VoxbloxMap& map,
  const std::string& method_name,
  const SensorModel& sensor_lidar,
  double threshold,
  int radius_max_surface,
  float dist_min,
  float dist_max,
  bool occlusion,
  float angle_low,
  float angle_high,
  const BoundingBox& bb,
  const std::vector<QualityArea>& qa, 
  bool verbose
) 
{
  m_map = map ;
  m_method_type = method_name ;
  p_ray_step_ = m_map.getVoxelSize() ;
  m_dist_max = dist_max;
  m_dist_min = dist_min ;
  m_occlusion = occlusion ;
  m_bb = bb;
  m_qa = qa ; 
  verbose_ = verbose;

  m_angle_high = angle_high ; 
  m_angle_low = angle_low ;
  p_downsampling_factor_ = 1.0 ;
  sensor_model_ = sensor_lidar ; 
  m_threshold_known = threshold ; 
  distance_surface_radius_max_ = radius_max_surface ; 

  // Downsample to voxel size resolution at max range
  c_res_x_ = std::min(static_cast<int>(std::ceil(
                          sensor_lidar.p_ray_length_ * sensor_lidar.p_fov_x_ /
                          (m_map.getVoxelSize() * p_downsampling_factor_))),
                      sensor_lidar.p_resolution_x_);
  c_res_y_ = std::min(static_cast<int>(std::ceil(
                          sensor_lidar.p_ray_length_ * sensor_lidar.p_fov_y_ /
                          (m_map.getVoxelSize() * p_downsampling_factor_))),
                      sensor_lidar.p_resolution_y_);

  // Determine number of splits + split distances
  c_n_sections_ =
      std::floor(static_cast<double>(std::log2(std::min(c_res_x_, c_res_y_))));
  c_split_widths_.push_back(0);
  for (int i = 0; i < c_n_sections_; ++i) {
    c_split_widths_.push_back(std::pow(2, i));
    c_split_distances_.push_back(sensor_lidar.p_ray_length_ /
                                 std::pow(2.0, static_cast<double>(i)));
  }
  c_split_distances_.push_back(0.0);
  std::reverse(c_split_distances_.begin(), c_split_distances_.end());
  std::reverse(c_split_widths_.begin(), c_split_widths_.end());

  float vs = m_map.getVoxelSize() ;  // voxel temp variable
  //Frontiers data 
  m_frontier6 = false;
  if(m_frontier6 == true){
    c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
    c_neighbor_voxels_[1] = Eigen::Vector3d(-vs, 0, 0);
    c_neighbor_voxels_[2] = Eigen::Vector3d(0, vs, 0);
    c_neighbor_voxels_[3] = Eigen::Vector3d(0, -vs, 0);
    c_neighbor_voxels_[4] = Eigen::Vector3d(0, 0, vs);
    c_neighbor_voxels_[5] = Eigen::Vector3d(0, 0, -vs);
  }
  else{
    // c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
    // c_neighbor_voxels_[1] = Eigen::Vector3d(vs, vs, 0);
    // c_neighbor_voxels_[2] = Eigen::Vector3d(vs, -vs, 0);
    // c_neighbor_voxels_[3] = Eigen::Vector3d(vs, 0, vs);
    // c_neighbor_voxels_[4] = Eigen::Vector3d(vs, vs, vs);
    // c_neighbor_voxels_[5] = Eigen::Vector3d(vs, -vs, vs);
    // c_neighbor_voxels_[6] = Eigen::Vector3d(vs, 0, -vs);
    // c_neighbor_voxels_[7] = Eigen::Vector3d(vs, vs, -vs);
    // c_neighbor_voxels_[8] = Eigen::Vector3d(vs, -vs, -vs);
    // c_neighbor_voxels_[9] = Eigen::Vector3d(0, vs, 0);
    // c_neighbor_voxels_[10] = Eigen::Vector3d(0, -vs, 0);
    // c_neighbor_voxels_[11] = Eigen::Vector3d(0, 0, vs);
    // c_neighbor_voxels_[12] = Eigen::Vector3d(0, vs, vs);
    // c_neighbor_voxels_[13] = Eigen::Vector3d(0, -vs, vs);
    // c_neighbor_voxels_[14] = Eigen::Vector3d(0, 0, -vs);
    // c_neighbor_voxels_[15] = Eigen::Vector3d(0, vs, -vs);
    // c_neighbor_voxels_[16] = Eigen::Vector3d(0, -vs, -vs);
    // c_neighbor_voxels_[17] = Eigen::Vector3d(-vs, 0, 0);
    // c_neighbor_voxels_[18] = Eigen::Vector3d(-vs, vs, 0);
    // c_neighbor_voxels_[19] = Eigen::Vector3d(-vs, -vs, 0);
    // c_neighbor_voxels_[20] = Eigen::Vector3d(-vs, 0, vs);
    // c_neighbor_voxels_[21] = Eigen::Vector3d(-vs, vs, vs);
    // c_neighbor_voxels_[22] = Eigen::Vector3d(-vs, -vs, vs);
    // c_neighbor_voxels_[23] = Eigen::Vector3d(-vs, 0, -vs);
    // c_neighbor_voxels_[24] = Eigen::Vector3d(-vs, vs, -vs);
    // c_neighbor_voxels_[25] = Eigen::Vector3d(-vs, -vs, -vs);



    c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
    c_neighbor_voxels_[1] = Eigen::Vector3d(-vs, 0, 0);
    c_neighbor_voxels_[2] = Eigen::Vector3d(0, vs, 0);
    c_neighbor_voxels_[3] = Eigen::Vector3d(0, -vs, 0);
    c_neighbor_voxels_[4] = Eigen::Vector3d(0, 0, vs);
    c_neighbor_voxels_[5] = Eigen::Vector3d(0, 0, -vs);
    
    c_neighbor_voxels_[6] = Eigen::Vector3d(vs, 0, -vs);
    c_neighbor_voxels_[7] = Eigen::Vector3d(vs, vs, -vs);
    c_neighbor_voxels_[8] = Eigen::Vector3d(vs, -vs, -vs);
    c_neighbor_voxels_[9] = Eigen::Vector3d(vs, vs, 0);
    c_neighbor_voxels_[10] = Eigen::Vector3d(vs, -vs, 0);
    c_neighbor_voxels_[11] = Eigen::Vector3d(vs, 0, vs);
    c_neighbor_voxels_[12] = Eigen::Vector3d(0, vs, vs);
    c_neighbor_voxels_[13] = Eigen::Vector3d(0, -vs, vs);
    c_neighbor_voxels_[14] = Eigen::Vector3d(vs, vs, vs);
    c_neighbor_voxels_[15] = Eigen::Vector3d(0, vs, -vs);
    c_neighbor_voxels_[16] = Eigen::Vector3d(0, -vs, -vs);
    c_neighbor_voxels_[17] = Eigen::Vector3d(vs, -vs, vs);
    c_neighbor_voxels_[18] = Eigen::Vector3d(-vs, vs, 0);
    c_neighbor_voxels_[19] = Eigen::Vector3d(-vs, -vs, 0);
    c_neighbor_voxels_[20] = Eigen::Vector3d(-vs, 0, vs);
    c_neighbor_voxels_[21] = Eigen::Vector3d(-vs, vs, vs);
    c_neighbor_voxels_[22] = Eigen::Vector3d(-vs, -vs, vs);
    c_neighbor_voxels_[23] = Eigen::Vector3d(-vs, 0, -vs);
    c_neighbor_voxels_[24] = Eigen::Vector3d(-vs, vs, -vs);
    c_neighbor_voxels_[25] = Eigen::Vector3d(-vs, -vs, -vs);
  } 


}

// void ViewEvaluator::getVisibleVoxels_camera(const ViewCandidate& vc){
//   // Naive ray-casting
//   Eigen::Vector3d camera_direction;
//   Eigen::Vector3d direction;
//   Eigen::Vector3d current_position;
//   Eigen::Vector3d voxel_center;
//   for (int i = 0; i < c_res_x_; ++i) {
//     for (int j = 0; j < c_res_y_; ++j) {
//       CameraModel::getDirectionVector(
//           &camera_direction,
//           static_cast<double>(i) / (static_cast<double>(c_res_x_) - 1.0),
//           static_cast<double>(j) / (static_cast<double>(c_res_y_) - 1.0));
//       direction = orientation * camera_direction;
//       double distance = 0.0;
//       while (distance < p_ray_length_) {
//         current_position = position + distance * direction;
//         distance += p_ray_step_;

//         // Check voxel occupied
//         if (map_.getVoxelState(current_position) ==
//             map::OccupancyMap::OCCUPIED) {
//           break;
//         }

//         // Add point (duplicates are handled in
//         // CameraModel::getVisibleVoxelsFromTrajectory)
//         m_map.getVoxelCenter_TSDF(&voxel_center, current_position);
//         result.push_back(voxel_center);
//       }
//     }
//   }
//   return true

// }

float ViewEvaluator::evaluate_view_angular(const ViewCandidate& vc, const Eigen::Vector3d& robot_pos, const Eigen::Vector3d& vel){
  float result = -100 ; 
  Eigen::Vector3d view_pos = Eigen::Vector3d(vc.x, vc.y, vc.z) ;
  Eigen::Vector3d comp = (view_pos - robot_pos) / (view_pos - robot_pos).norm() ;
  
  Eigen::Vector3d vel_temp ;
  if (vel.norm() != 0 ){
      vel_temp = vel / vel.norm() ; 
  }
  result = vel_temp.transpose() * comp ;

  result = acos( result ) ; 
  result = (M_PI - result) / M_PI ; 
  return result ;  
}

float ViewEvaluator::evaluate_distance_angle_cost( float value_angular, float dist_min_frontiers, float dist_view ){

  if (dist_view > sensor_model_.p_ray_length_){ // if view candidate is out of a sphere of max sensor range, the view is penalized (lowest dist/ang value)
    return 0;
  }

  return value_angular * (dist_min_frontiers / dist_view ) ;
  //return value_angular * exp( - ( dist_view ) ) ; 
}


void ViewEvaluator::getVisibleVoxels_LIDAR(
    std::vector<Eigen::Vector3d>* result, const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation) {
  // Setup ray table (contains at which segment to start, -1 if occluded
  ray_table_ = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);

  // Ray-casting
  Eigen::Vector3d camera_direction;
  Eigen::Vector3d direction;
  Eigen::Vector3d current_position;
  Eigen::Vector3d voxel_center;
  double distance;
  bool cast_ray;
  double map_distance;
  for (int i = 0; i < c_res_x_; ++i) {
    for (int j = 0; j < c_res_y_; ++j) {
      int current_segment = ray_table_(i, j);  // get ray starting segment
      if (current_segment < 0) {
        continue;  // already occluded ray
      }
      sensor_model_.getDirectionVector_LIDAR(
          &camera_direction,
          static_cast<double>(i) / (static_cast<double>(c_res_x_) - 1.0),
          static_cast<double>(j) / (static_cast<double>(c_res_y_) - 1.0));
      direction = orientation * camera_direction; // direction of the lidar in general * current orientation of the sensor
      distance = c_split_distances_[current_segment];
      cast_ray = true;
      while (cast_ray) {
        // iterate through all splits (segments)
        while (distance < c_split_distances_[current_segment + 1]) {
          current_position = position + distance * direction;
          distance += p_ray_step_;

          // Add point (duplicates are handled in
          // CameraModel::getVisibleVoxelsFromTrajectory)
          m_map.getVoxelCenter_TSDF(&voxel_center, current_position);
          result->push_back(voxel_center);

          // Check voxel occupied 
          if (m_map.getVoxelState_TSDF(current_position , m_threshold_known) ==
              voxblox_map::VoxbloxMap::OCCUPIED) {
            // Occlusion, mark neighboring rays as occluded
            markNeighboringRays(i, j, current_segment, -1);
            cast_ray = false;
            break;
          }
        }
        if (cast_ray) {
          current_segment++;
          if (current_segment >= c_n_sections_) {
            cast_ray = false;  // done
          } else {
            // update ray starts of neighboring rays
            markNeighboringRays(i, j, current_segment - 1, current_segment);
          }
        }
      }
    }
  }
  //return true;
}


void ViewEvaluator::markNeighboringRays(int x, int y, int segment,
                                                  int value) {
  // Set all nearby (towards bottom right) ray starts, depending on the segment
  // depth, to a value.
  for (int i = x; i < std::min(c_res_x_, x + c_split_widths_[segment]); ++i) {
    for (int j = y; j < std::min(c_res_y_, y + c_split_widths_[segment]); ++j) {
      ray_table_(i, j) = value;
    }
  }
}



float ViewEvaluator::inverseRayCast(const ViewCandidate& vc,const std::vector<Eigen::Vector3d>& frontiers_set, int& nb_frontiers_visible, std::unordered_map<Eigen::Vector3d, int, Vector3dHash, Vector3dEqual>& visible_frontiers, bool save_reached_frontiers ){
  int result = 0 ; 
  Eigen::Vector3d frontier ;
  Eigen::Vector3d pos( vc.x, vc.y, vc.z); 
  ViewCandidate vc_2 ;
  float temp_distance = -1 ;
  float nb_voxel_distance = -10000;
  float total_distance = 0 ; 
  float nb_unknown_vox = -1;
  float average_dist_frontier = 0 ;
  int nb_frontiers_examined = 0 ;

  float distance_ponderation = 0 ; 
  float frontier_quality_min = 0; 
  float frontier_quality_max = 0 ; 

  float temp_dist_min = 0 ;
  float temp_dist_max = 0 ;

  if ( !lineOfSightCheck(vc, nb_unknown_vox) ){
    return -2;
  }

  float range = sensor_model_.p_ray_length_ ; 
  for(int i =0; i< frontiers_set.size();i++){

    frontier = frontiers_set[i];

    if( (frontier - pos).norm() < range ){
      nb_frontiers_examined = nb_frontiers_examined +1;

      vc_2 = vc ; //deep copy only for simple types
      vc_2.o_x = frontier.x();
      vc_2.o_y = frontier.y();
      vc_2.o_z = frontier.z(); 

      if( lineOfSightCheck(vc_2, nb_unknown_vox) ){
        result = result +1 ;
        temp_distance = (frontier - pos).norm() ;
        nb_voxel_distance = temp_distance * m_map.getVoxelSize() ;
        average_dist_frontier = average_dist_frontier + temp_distance ; 

        if( save_reached_frontiers == true){
          visible_frontiers[ frontier ] = 0 ; 
        }
        
        //if multi resolution
        if( m_qa.size() !=0) {

          if (get_quality_target_for_frontier(m_qa, frontier, temp_dist_min, temp_dist_max) ){
            frontier_quality_min = temp_dist_min ; 
            frontier_quality_max = temp_dist_max ; 
          }
          else{
            frontier_quality_min = m_dist_min; 
            frontier_quality_max = m_dist_max; 
          }

          distance_ponderation = ponder_frontier_by_distance_multi_resolution( temp_distance, frontier_quality_min, frontier_quality_max ) ; 
        }
        else{
          distance_ponderation = ponder_frontier_by_distance( temp_distance ) ; 
        }


        if ( m_occlusion ){
          total_distance = total_distance + distance_ponderation * pow(2, -( nb_unknown_vox/ nb_voxel_distance) ); 
        }
        else{
          total_distance = total_distance + distance_ponderation ; 

        }



      }


    }



  }
  average_dist_frontier= average_dist_frontier/result;
  ROS_INFO_COND(verbose_,"AVERAGE DISTANCE OF GOAL IS %f", average_dist_frontier);
  ROS_INFO_COND(verbose_," Frontiers visible/good angle are  %d on %d",result, nb_frontiers_examined );
  //TO UNCOMMENT IF REVERTED TO NON NORMALIZED : 
  //total_distance = ( total_distance / frontiers_set.size() ) ;
  nb_frontiers_visible = result;
  result = (result / frontiers_set.size()); 
  return total_distance;
  //return result;

}

float ViewEvaluator::inverseRayCast_no_distance(const ViewCandidate& vc,const std::vector<Eigen::Vector3d>& frontiers_set, int& nb_frontiers_visible, std::unordered_map<Eigen::Vector3d, int, Vector3dHash, Vector3dEqual>& visible_frontiers, bool save_reached_frontiers ){
  int result = 0 ; 
  Eigen::Vector3d frontier ;
  Eigen::Vector3d pos( vc.x, vc.y, vc.z); 
  ViewCandidate vc_2 ;
  float temp_distance = -1 ;
  float nb_voxel_distance = -10000;
  float total_distance = 0 ; 
  float nb_unknown_vox = -1;
  float average_dist_frontier = 0 ;
  int nb_frontiers_examined = 0 ;

  if ( !lineOfSightCheck(vc, nb_unknown_vox) ){
    return -2;
  }

  float range = sensor_model_.p_ray_length_ ; 
  for(int i =0; i< frontiers_set.size();i++){

    frontier = frontiers_set[i];

    if( (frontier - pos).norm() < range ){
      nb_frontiers_examined = nb_frontiers_examined +1;

      vc_2 = vc ; //deep copy only for simple types
      vc_2.o_x = frontier.x();
      vc_2.o_y = frontier.y();
      vc_2.o_z = frontier.z(); 

      if( lineOfSightCheck(vc_2, nb_unknown_vox) ){
        result = result +1 ;
      }


    }



  }

  nb_frontiers_visible = result;
  result = (result / frontiers_set.size()); 
  return result;

}


//Given a set of points, evaluate using inverse raycasting the pondered (based on the distance of observation) number of these points from a ViewCandidate 
float ViewEvaluator::inverseRayCast_simple(const ViewCandidate& vc,const std::vector<Eigen::Vector3d>& to_observe_set, int& nb_observed){
  int result = 0 ; 
  Eigen::Vector3d frontier ;
  Eigen::Vector3d pos( vc.x, vc.y, vc.z); 
  ViewCandidate vc_2 ;
  float temp_distance = -1 ;
  float nb_voxel_distance = -10000;
  float total_distance = 0 ; 
  float nb_unknown_vox = -1;
  float average_dist_frontier = 0 ;
  int nb_points_examined = 0 ;

  float distance_ponderation = 0 ; 

  if ( !lineOfSightCheck(vc, nb_unknown_vox) ){
    return -2;
  }

  float range = sensor_model_.p_ray_length_ ; 
  for(int i =0; i< to_observe_set.size();i++){

    frontier = to_observe_set[i];

    if( (frontier - pos).norm() < range ){
      nb_points_examined = nb_points_examined +1;

      vc_2 = vc ; //deep copy only for simple types
      vc_2.o_x = frontier.x();
      vc_2.o_y = frontier.y();
      vc_2.o_z = frontier.z(); 

      if( lineOfSightCheck(vc_2, nb_unknown_vox) ){
        result = result +1 ;
        temp_distance = (frontier - pos).norm() ;
        nb_voxel_distance = temp_distance * m_map.getVoxelSize() ;
        average_dist_frontier = average_dist_frontier + temp_distance ; 


        distance_ponderation = ponder_frontier_by_distance( temp_distance ) ; 

        if ( m_occlusion ){
          total_distance = total_distance + distance_ponderation * pow(2, -( nb_unknown_vox/ nb_voxel_distance) ); 
        }
        else{
          total_distance = total_distance + distance_ponderation ; 

        }



      }


    }



  }
  average_dist_frontier= average_dist_frontier/result;
  ROS_INFO_COND(verbose_,"AVERAGE DISTANCE OF GOAL IS %f", average_dist_frontier);
  ROS_INFO_COND(verbose_," Points visible/good angle are  %d on %d",result, nb_points_examined );
  nb_observed = result;
  result = (result / to_observe_set.size()); 
  return total_distance;
  //return result;

}

float ViewEvaluator::ponder_frontier_by_distance(float frontier_dist_to_vc){
  if ( frontier_dist_to_vc < m_dist_min){
    return 0.5;
  }

  if( frontier_dist_to_vc >= m_dist_min && frontier_dist_to_vc <= m_dist_max ){
    return 1.0 ; 
  }

  if( frontier_dist_to_vc > m_dist_max){
    return (0.5 - ( ( (frontier_dist_to_vc - m_dist_max) / (sensor_model_.p_ray_length_ - m_dist_max) )* 0.5 ) );
  }


}


float ViewEvaluator::ponder_frontier_by_distance_multi_resolution(float frontier_dist_to_vc, float dist_min, float dist_max){
  if ( frontier_dist_to_vc < dist_min){
    return 0.5;
  }

  if( frontier_dist_to_vc >= dist_min && frontier_dist_to_vc <= dist_max ){
    return 1.0 ; 
  }

  if( frontier_dist_to_vc > dist_max){
    return (0.5 - ( ( (frontier_dist_to_vc - dist_max) / (sensor_model_.p_ray_length_ - dist_max) )* 0.5 ) );
  }


}

bool ViewEvaluator::lineOfSightCheck(const ViewCandidate& vc, float& nb_unknown_voxels ){
  Eigen::Vector3d frontier(vc.o_x, vc.o_y, vc.o_z);
  Eigen::Vector3d view( vc.x, vc.y, vc.z);
  Eigen::Vector3d direction = (view - frontier).normalized() ; 
  float distance = (view - frontier).norm() ; //not sure at all, not correct (my count is squared of this norm)
  Eigen::Vector3d new_pos = frontier ; 
  Eigen::Vector3d old_pos = frontier ; // deepcopy ? yes. 
  char state ;
  float unknown_vox = 0;
  float angle = 0;
  bool isAngleOk = false;

  //check angle
  isAngleOk = verify_angle(frontier, vc, angle, m_angle_low, m_angle_high ); 
  if ( isAngleOk == false ){
    // ROS_INFO_COND(verbose_," Angle is %f : refused ", angle);
    return false;
  }
  // ROS_INFO_COND(verbose_," Angle is %f : accepted ", angle);

  for(int i=0; i< ( 2* distance) ; i++  ){ // while ?

    new_pos = new_pos + direction ; 

    if( (frontier - new_pos).norm() > distance || (frontier - new_pos).norm() > sensor_model_.p_ray_length_ ){ 
      break ;
    } 

    // only checks voxel status, if we changed voxels ie one of the dimension has a change > 1
    if ( std::abs(old_pos.x() - new_pos.x()) >= 1 || std::abs(old_pos.y() - new_pos.y()) >= 1 || std::abs(old_pos.z() - new_pos.z()) >= 1 ){ 

      old_pos = new_pos;
      state = m_map.getVoxelState_TSDF(new_pos, m_threshold_known ); 


      /// topology occlusion check



      // topology occlusion check end

      if(  state == voxblox_map::VoxbloxMap::UNKNOWN){
        unknown_vox = unknown_vox +1;
      } 
      if( state == voxblox_map::VoxbloxMap::OCCUPIED ){
        return false;
      }

    }


  }
  nb_unknown_voxels = unknown_vox;
  return true;
}



bool ViewEvaluator::isSurfaceFrontier_TSDF(const Eigen::Vector3d& voxel, Eigen::Vector3d& unknown_vox, unsigned char& current_state){
//bool ViewEvaluator::isSurfaceFrontier_TSDF(const Eigen::Vector3d& voxel){
  unsigned char voxel_state;
  //unsigned char current_state;
  bool is_surface = false;
  bool close_unknown = false;
  bool close_empty = false;

  if ( isCorrectPos(voxel, m_bb) == false ){
    return false;
  }

  current_state = m_map.getVoxelState_TSDF(voxel, 0);
  if( (current_state == voxblox_map::VoxbloxMap::OCCUPIED) && ( m_map.getVoxelDistance_TSDF(voxel) >=0 ) ){
      is_surface = true;
  } 
  else{
    return false;
  }

  for (int i = 0; i < 26; ++i) {

    voxel_state = m_map.getVoxelState_TSDF(voxel + c_neighbor_voxels_[i], 0);
    // ROS_INFO_COND(verbose_," VOXEL STATE IS %d", voxel_state);
    if ( i<6 && voxel_state == voxblox_map::VoxbloxMap::UNKNOWN) {
    //if (voxel_state == voxblox_map::VoxbloxMap::UNKNOWN) {
      close_unknown = true;
      unknown_vox = voxel + c_neighbor_voxels_[i] ;
      // ROS_INFO_COND(verbose_, "FOUND UNKNOWN %f %f %f", unknown_vox.x(), unknown_vox.y(), unknown_vox.z());
      //return true;
    }

    if (voxel_state == voxblox_map::VoxbloxMap::FREE) {
      close_empty = true;
      //return true;
    }




  }

  //if( is_surface && close_unknown ){
  if( is_surface && close_unknown && close_empty ){
    return true;
  }
  return false;

}

bool ViewEvaluator::isLowConfidenceVoxel(const Eigen::Vector3d& voxel){
  unsigned char current_state ; 
  float voxel_weight = 0 ;
  float voxel_dist = 0 ; 
  float temp_dist_min = 0 ;
  float temp_dist_max = 0 ;

  voxel_weight = m_map.getVoxelWeight_TSDF(voxel); 
  current_state = m_map.getVoxelState_TSDF(voxel, 0);
  voxel_dist = m_map.getVoxelDistance_TSDF(voxel) ; 

  float minimum_weight = 0 ; 

  //if multi resolution
  if( (m_qa.size() > 0) && (get_quality_target_for_frontier(m_qa, voxel, temp_dist_min, temp_dist_max)) ){

    minimum_weight = 1.0 / (temp_dist_max * temp_dist_max) ; 

  }
  else{

    //if single resolution
    minimum_weight = 1.0 / ( m_dist_max * m_dist_max) ; 

  }

  if( (voxel_weight >= minimum_weight) || current_state!= voxblox_map::VoxbloxMap::OCCUPIED ){
    return false;
  }

  if( voxel_dist < - m_map.getVoxelSize() ){
    return false ; 
  }


  return true;
  

}


float ViewEvaluator::getMinMaxViewDistance(std::vector<ViewCandidate> views, const Eigen::Vector3d& current_pos_vector, float& max_dist){

  float temp_distance = -1;
  float temp_max_distance = -1;
  float max_distance = std::numeric_limits<float>::min() ;
  float min_distance = std::numeric_limits<float>::max();
  for(int i=0;i< views.size();i++){

    ViewCandidate view = views[i] ;
    Eigen::Vector3d view_vector = Eigen::Vector3d( view.x, view.y, view.z);
    temp_distance = (view_vector - current_pos_vector).norm() ;

    if ( temp_distance < min_distance ){
      min_distance = temp_distance ; 
    }

    if ( temp_distance > max_distance){
      max_distance = temp_distance;
    }
    
    
    
  }
  max_dist = max_distance;
  return min_distance;

}




ViewEvaluator::~ViewEvaluator()
{
}
