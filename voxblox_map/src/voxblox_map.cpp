#include <voxblox_map/voxblox_map.h>
#include <limits>


//#include "active_3d_planning_core/data/system_constraints.h"

namespace voxblox_map {



// VoxbloxMap::VoxbloxMap(int collision_radius) : {
//     // create an esdf server
//   ros::NodeHandle nh("");
//   ros::NodeHandle nh_private("~");
//   esdf_server_.reset(new voxblox::EsdfServer(nh, nh_private));
//   m_collision_radius = collision_radius;
//   esdf_server_->setTraversabilityRadius(
//       collision_radius);

//   // cache constants
//   c_voxel_size_ = esdf_server_->getEsdfMapPtr()->voxel_size();
//   c_block_size_ = esdf_server_->getEsdfMapPtr()->block_size();
//   c_maximum_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(nh_private)
//                           .max_weight;  // direct access is not exposed
// }


VoxbloxMap::VoxbloxMap(FloatingPoint voxel_size, size_t voxels_per_side) {

    layer_tsdf_.reset(new voxblox::Layer<voxblox::TsdfVoxel>(voxel_size, voxels_per_side));
    layer_esdf_.reset(new voxblox::Layer<voxblox::EsdfVoxel>(voxel_size, voxels_per_side));

    esdf_map_pointer = nullptr;
    tsdf_map_pointer = nullptr;

    esdf_map_pointer.reset(new voxblox::EsdfMap(layer_esdf_));
    tsdf_map_pointer.reset(new voxblox::TsdfMap(layer_tsdf_));

    m_distance_threshold = 0;
    m_confidence_threshold = 0 ;
    c_voxel_size_ = voxel_size;
    c_block_size_ = voxels_per_side;

}

void VoxbloxMap::addTSDFLayer(voxblox::Layer<voxblox::TsdfVoxel>::Ptr new_layer) {

  tsdf_map_pointer = nullptr;
  tsdf_map_pointer.reset(new voxblox::TsdfMap(new_layer));

}


// voxblox::EsdfServer& VoxbloxMap::getESDFServer() { return *esdf_server_; }


//use ESDF
bool VoxbloxMap::isTraversable_ESDF(const Eigen::Vector3d& position,
                               const Eigen::Quaterniond& orientation) {
  double distance = 0.0;
  if (esdf_map_pointer->getDistanceAtPosition(position,
                                                           &distance)) {
    // This means the voxel is observed
    return (distance > m_collision_radius); //CONSIDERS UNIQUE RADIUS !!! TO CHANGE
  }
  return false;
}
//getDisanceAtPosition unavailable in TSDF_map.h

double VoxbloxMap::getDistancePrecise_ESDF(const Eigen::Vector3d& position) {
  double distance = 0.0;
  if (esdf_map_pointer->getDistanceAtPosition(position,
                                &distance)) {
    return distance;
  }
  //return std::numeric_limits<double>::lowest() ; 
  return 0 ;
}


bool VoxbloxMap::isObserved_ESDF(const Eigen::Vector3d& point) {
  return esdf_map_pointer->isObserved(point);
}
// isObserved is unavailable in TSDF_map.h


// get occupancy - use ESDF
//inappropriate !!! 
unsigned char VoxbloxMap::getVoxelState_ESDF(const Eigen::Vector3d& point) {
  double distance = 0.0;
  if (esdf_map_pointer->getDistanceAtPosition(point, &distance)) {
    // This means the voxel is observed
    if (distance < c_voxel_size_) {
      return VoxbloxMap::OCCUPIED;
    } else {
      return VoxbloxMap::FREE;
    }
  } else {
    return VoxbloxMap::UNKNOWN;
  }
}
// get occupancy - use TSDF
//getDistanceAtPosition unavailable in TSDF_map.h

//THIS FUNCTION SHOULD BE USED FOR FRONTIERS DETECTION !! 
unsigned char VoxbloxMap::getVoxelState_TSDF(const Eigen::Vector3d& point, double t_threshold) {
  double distance = getVoxelDistance_TSDF(point);
  double weight = getVoxelWeight_TSDF(point);
  double threshold = t_threshold ; 
  if (weight > threshold) {
    // This means the voxel is observed
    if (distance < c_voxel_size_) {
      return VoxbloxMap::OCCUPIED;
    } else {
      return VoxbloxMap::FREE;
    }
  } else {
    return VoxbloxMap::UNKNOWN;
  }
}


// get voxel size 
double VoxbloxMap::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point - use ESDF
bool VoxbloxMap::getVoxelCenter_ESDF(Eigen::Vector3d* center,
                                const Eigen::Vector3d& point) {
  voxblox::BlockIndex block_id = esdf_map_pointer
                                     ->getEsdfLayerPtr()
                                     ->computeBlockIndexFromCoordinates(
                                         point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_)
                .cast<double>();
  voxblox::VoxelIndex voxel_id =
      voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
          (point - *center).cast<voxblox::FloatingPoint>(),
          1.0 / c_voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_)
                 .cast<double>();
  return true;
}

// get the center of a voxel from input point - use TSDF
bool VoxbloxMap::getVoxelCenter_TSDF(Eigen::Vector3d* center,
                                const Eigen::Vector3d& point) {
  voxblox::BlockIndex block_id = tsdf_map_pointer
                                     ->getTsdfLayerPtr()
                                     ->computeBlockIndexFromCoordinates(
                                         point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_)
                .cast<double>();
  voxblox::VoxelIndex voxel_id =
      voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
          (point - *center).cast<voxblox::FloatingPoint>(),
          1.0 / c_voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_)
                 .cast<double>();
  return true;
}

// get the stored TSDF distance - use TSDF
double VoxbloxMap::getVoxelDistance_TSDF(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
      tsdf_map_pointer
          ->getTsdfLayerPtr()
          ->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::TsdfVoxel* tsdf_voxel =
        block->getVoxelPtrByCoordinates(voxblox_point);
    if (tsdf_voxel) {
      return tsdf_voxel->distance;
    }
  }
  return 0.0;
}

// get the stored ESDF distance - use ESDF
double VoxbloxMap::getVoxelDistance_ESDF(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::EsdfVoxel>::Ptr block =
      esdf_map_pointer
          ->getEsdfLayerPtr()
          ->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::EsdfVoxel* esdf_voxel =
        block->getVoxelPtrByCoordinates(voxblox_point);
    if (esdf_voxel) {
      return esdf_voxel->distance;
    }
  }
  return 0.0;
}

// get the stored weight - use TSDF
double VoxbloxMap::getVoxelWeight_TSDF(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
      tsdf_map_pointer
          ->getTsdfLayerPtr()
          ->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::TsdfVoxel* tsdf_voxel =
        block->getVoxelPtrByCoordinates(voxblox_point);
    if (tsdf_voxel) {
      return tsdf_voxel->weight;
    }
  }
  return 0.0;
}
//ESDF Voxel has no weight


//CREATED BUT SHOULD NOT BE USED FOR VIEW GENERATION
// double VoxbloxMap::getVoxelGradient_TSDF(const Eigen::Vector3d& point) {
//   voxblox::Point voxblox_point(point.x(), point.y(), point.z());
//   voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
//       tsdf_map_pointer
//           ->getTsdfLayerPtr()
//           ->getBlockPtrByCoordinates(voxblox_point);
//   if (block) {
//     voxblox::TsdfVoxel* tsdf_voxel =
//         block->getVoxelPtrByCoordinates(voxblox_point);
//     if (tsdf_voxel) {
//       return tsdf_voxel->gradient;
//     }
//   }
//   return 0.0;
// }




// get the maximum allowed weight (return 0 if using uncapped weights)
double VoxbloxMap::getMaximumWeight() { return c_maximum_weight_; }

double VoxbloxMap::evaluation_TSDF(float truncationdist, float& maxx, float& minn, int &number_voxels, const BoundingBox& bb, float weight_saturation){
  VoxbloxMap empty_map(0,0);
  return evaluation_TSDF(truncationdist, maxx, minn, number_voxels, bb, weight_saturation, empty_map);

}


//evaluate the whole produced map 
double VoxbloxMap::evaluation_TSDF(float truncationdist, float& maxx, float& minn, int &number_voxels, const BoundingBox& bb, float weight_saturation, VoxbloxMap groundtruth){
  double result = 0.0;
  int voxel_count = 0 ;
  float voxeldistance = -100;
  float min_weight = FLT_MAX;
  float max_weight = - 1000000000;
  bool isInBB = false ; 
  Eigen::Vector3d voxel_vect ; 
  float distance_truth = 0;


  voxblox::BlockIndexList blocks;
  tsdf_map_pointer->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = tsdf_map_pointer->getTsdfLayerPtr()->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = tsdf_map_pointer->getTsdfLayerPtr()->getBlockByIndex(index);

    voxblox::Point origin = block.origin();

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
          ++linear_index) {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);

      voxel_vect = Eigen::Vector3d( coord.x(), coord.y(), coord.z() );  
      isInBB = isCorrectPos(voxel_vect, bb);
      if( !isInBB ){
        continue;
      }

      //Test if voxel is observed
      voxeldistance = voxel.distance;
      if( groundtruth.c_voxel_size_ != 0 ){
        distance_truth= groundtruth.getVoxelDistance_TSDF(voxel_vect);
      }

      if( ( abs(voxeldistance) < truncationdist) && ( abs(voxeldistance - distance_truth) < sqrt( 2.0 * c_voxel_size_ * c_voxel_size_ ) )  ){
      //if(voxeldistance > 0 ){
        
        //if( voxeldistance < truncationdist){
          //if closes to the surface sums its weights
          if(voxel.weight > weight_saturation){
            result = result + weight_saturation ;
          }
          else{
            result = result + voxel.weight ;
          }

          voxel_count = voxel_count + 1;

          if(voxel.weight > max_weight){
            max_weight = voxel.weight;
          }
          if(voxel.weight < min_weight){
            min_weight = voxel.weight;
          }
        //}
      } 

    }

  }
  maxx = max_weight;
  minn = min_weight;
  number_voxels = voxel_count;
  return result;

}

//evaluate the whole produced map 
double VoxbloxMap::countvoxelsinmap_TSDF(const BoundingBox& bb, int truncation_nb){
  double result = 0.0;
  int voxel_count = 0 ;
  float weight = -100;
  float dist = - 100000 ; 
  bool isInBB = false ; 
  Eigen::Vector3d voxel_vect ;

  voxblox::BlockIndexList blocks;
  tsdf_map_pointer->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = tsdf_map_pointer->getTsdfLayerPtr()->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = tsdf_map_pointer->getTsdfLayerPtr()->getBlockByIndex(index);

    voxblox::Point origin = block.origin();

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
          ++linear_index) {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);

      //Test if in bounding box
      voxel_vect = Eigen::Vector3d( coord.x(), coord.y(), coord.z() ); 
      isInBB = isCorrectPos(voxel_vect, bb);
      if( !isInBB ){
        continue;
      }

      //Test if voxel is observed
      weight = voxel.weight;
      dist = voxel.distance ; 
      //voxel_count = voxel_count +1 ; //to uncomment only to assess full map size
      //if( (weight > 0) && (abs(dist) < (truncation_nb* c_voxel_size_)) ){ //test around the surface (negative accepted)
      if( (weight > 0) && (dist <= (truncation_nb* c_voxel_size_)) && ( dist >= 0 )){ // original one
        
        voxel_count = voxel_count + 1;
        
      } 

    }

  }

  return voxel_count;

}

double VoxbloxMap::countvoxelsinmap_TSDF_corrected(const BoundingBox& bb, int truncation_nb, VoxbloxMap groundtruth, std::unordered_map<std::string, float>& result_map){
  double result = 0.0;
  int voxel_count = 0 ;
  float voxel_count_groundtruth = 0 ; 
  float voxels_too_far = 0 ; 
  float weight = -100;
  float weight_groundtruth = -100 ; 
  float weight_saturated = -100;
  float dist = - 100000 ; 
  float distance_truth = -100000;
  bool isInBB = false ; 
  Eigen::Vector3d voxel_vect ;
  Eigen::Vector3d voxel_rotated;
  float error = 0 ; 
  float weighted_error = 0 ; 

  float perfect_zeros = 0 ;



  voxblox::BlockIndexList blocks;
  tsdf_map_pointer->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = tsdf_map_pointer->getTsdfLayerPtr()->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = tsdf_map_pointer->getTsdfLayerPtr()->getBlockByIndex(index);

    voxblox::Point origin = block.origin();

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
          ++linear_index) {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);

      //Test if in bounding box
      voxel_vect = Eigen::Vector3d( coord.x(), coord.y(), coord.z() ); 
      isInBB = isCorrectPos(voxel_vect, bb);
      if( !isInBB ){
        continue;
      }

      //get_groundtruth value
      //voxel_rotated = Eigen::Vector3d( -coord.y(), coord.x(), coord.z() );  // test_voxel_rotated
      //distance_truth= groundtruth.getVoxelDistance_TSDF(voxel_rotated); // test_voxel_rotated
      distance_truth= groundtruth.getVoxelDistance_TSDF(voxel_vect);
      weight_groundtruth = groundtruth.getVoxelWeight_TSDF(voxel_vect);

      weight = voxel.weight;
      dist = voxel.distance ; 

      if( weight_groundtruth> 0 && weight > 0 && (distance_truth <= (truncation_nb* c_voxel_size_)) && ( distance_truth >= 0 ) ){
        voxel_count_groundtruth = voxel_count_groundtruth + 1 ;

        if(distance_truth == 0 ){
          perfect_zeros = perfect_zeros +1 ; 
        }
      } 
      //Test if voxel is observed




      //if( (weight > 0) && (abs(dist) < (truncation_nb* c_voxel_size_)) ){ //test around the surface (negative accepted)
      if( (weight > 0) && (dist <= (truncation_nb* c_voxel_size_)) && ( dist >= 0 )){ //original one

        error = error + ( distance_truth - dist ) * ( distance_truth - dist ) ; 
        if( weight > 1 ){
          weight_saturated = 1 ; 
        }
        else{
          weight_saturated = weight ; 
        }
        weighted_error = weighted_error + weight_saturated * ( distance_truth - dist ) * ( distance_truth - dist ) ;
        
        if( (abs(distance_truth - dist)) < sqrt( (3.0/2.0) * c_voxel_size_ * c_voxel_size_ ) ){ //ref from hardouin
        //if( (distance_truth < (truncation_nb* c_voxel_size_)) && ( distance_truth > 0 ) ){
          voxel_count = voxel_count + 1;
        }
        else{
          voxels_too_far = voxels_too_far +1 ; 
        }
        
      } 

    }

  }
  result_map["error"] = error ; 
  result_map["weighted error"] = weighted_error ; 
  result_map["total voxels"] = voxel_count + voxels_too_far;
  result_map["voxels covered"] = voxel_count ;
  result_map["voxels too far"] = voxels_too_far ; 
  //result_map["voxel surface groundtruth"] = voxel_count_groundtruth; 
  //result_map["perfect_zeros"] = perfect_zeros;
  return voxel_count;

}

double VoxbloxMap::count_surface_voxels(const BoundingBox& bb, int truncation_nb, VoxbloxMap groundtruth, std::unordered_map<std::string, float>& result_map){


  double result = 0.0;
  int surface_count = 0 ;
  float groundtruth_surface_count = 0 ; 
  float voxels_too_far = 0 ; 
  float weight = -100;
  float weight_groundtruth = -100 ; 
  float weight_neighbor = -1 ; 
  float dist = - 100000 ; 
  float distance_truth = -100000;
  float dist_neighbor = -1000 ; 
  bool isInBB = false ; 
  Eigen::Vector3d voxel_vect ;
  float error = 0 ; 

  Eigen::Vector3d c_neighbor_voxels_[26];

  float vs = getVoxelSize() ; 
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


  voxblox::BlockIndexList blocks;
  tsdf_map_pointer->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = tsdf_map_pointer->getTsdfLayerPtr()->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = tsdf_map_pointer->getTsdfLayerPtr()->getBlockByIndex(index);

    voxblox::Point origin = block.origin();

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
          ++linear_index) {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);

      //Test if in bounding box
      voxel_vect = Eigen::Vector3d( coord.x(), coord.y(), coord.z() ); 
      isInBB = isCorrectPos(voxel_vect, bb);
      if( !isInBB ){
        continue;
      }

    
      distance_truth= groundtruth.getVoxelDistance_TSDF(voxel_vect);
      weight_groundtruth = groundtruth.getVoxelWeight_TSDF(voxel_vect);
      weight = voxel.weight;
      dist = voxel.distance ; 

      if( weight > 0){


        //count groundtruth surfaces
        if((weight_groundtruth >0) && (distance_truth <= (truncation_nb* c_voxel_size_)) && ( distance_truth > 0 ) ){

          for(int i = 0; i< 26;i++){
        
            weight_neighbor = groundtruth.getVoxelWeight_TSDF( voxel_vect + c_neighbor_voxels_[i] ) ; 
            dist_neighbor = groundtruth.getVoxelDistance_TSDF( voxel_vect + c_neighbor_voxels_[i] ) ; 
            if( weight_neighbor > 0 && dist_neighbor < 0 ){
            
              groundtruth_surface_count = groundtruth_surface_count +1 ;
              break ; 
            }

          } 

        }




        //count voxel surfaces
        if((dist <= (truncation_nb* c_voxel_size_)) && ( dist > 0 ) ){

          for(int i = 0; i< 26;i++){
        
            weight_neighbor = getVoxelWeight_TSDF( voxel_vect + c_neighbor_voxels_[i] ) ; 
            dist_neighbor = getVoxelDistance_TSDF( voxel_vect + c_neighbor_voxels_[i] ) ; 
            if( weight_neighbor > 0 && dist_neighbor < 0 ){
            
              surface_count = surface_count +1 ;
              error = error + ( distance_truth - dist ) * ( distance_truth - dist ) ; 
              break ; 
            }

          } 

        }

                

      }


    } 

  }
  
  result_map["error"] = error ; 
  ROS_WARN(" Error 2 is %f", error ) ;
  result_map["groundtruth surface"] = groundtruth_surface_count ; 
  ROS_WARN(" GD 2 is %f", groundtruth_surface_count ) ;
  ROS_WARN(" Surface 2 is %d",surface_count ) ;

  return surface_count;
}

double VoxbloxMap::count_surface_voxels_no_GD(const BoundingBox& bb, int truncation_nb, float weight_min){


  int surface_count = 0 ;
  float weight = -100;
  float weight_neighbor = -1 ; 
  float dist = - 100000 ; 
  float dist_neighbor = -1000 ; 
  bool isInBB = false ; 
  Eigen::Vector3d voxel_vect ;
  float error = 0 ; 

  Eigen::Vector3d c_neighbor_voxels_[26];

  float vs = getVoxelSize() ; 
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


  voxblox::BlockIndexList blocks;
  tsdf_map_pointer->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = tsdf_map_pointer->getTsdfLayerPtr()->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = tsdf_map_pointer->getTsdfLayerPtr()->getBlockByIndex(index);

    voxblox::Point origin = block.origin();

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
          ++linear_index) {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);

      //Test if in bounding box
      voxel_vect = Eigen::Vector3d( coord.x(), coord.y(), coord.z() ); 
      isInBB = isCorrectPos(voxel_vect, bb);
      if( !isInBB ){
        continue;
      }
    
      weight = voxel.weight;
      dist = voxel.distance ; 

      if( weight > weight_min){


        //count voxel surfaces
        if((dist <= (truncation_nb* c_voxel_size_)) && ( dist > 0 ) ){

          for(int i = 0; i< 26;i++){
        
            weight_neighbor = getVoxelWeight_TSDF( voxel_vect + c_neighbor_voxels_[i] ) ; 
            dist_neighbor = getVoxelDistance_TSDF( voxel_vect + c_neighbor_voxels_[i] ) ; 
            if( weight_neighbor > 0 && dist_neighbor < 0 ){
            
              surface_count = surface_count +1 ;
              break ; 
            }

          } 

        }

                

      }


    } 

  }

    return surface_count;


}



bool VoxbloxMap::isValidPos(const Eigen::Vector3d& pos){
  if(pos.z() <=0){
    return false;
  }


  return true;

}


void VoxbloxMap::export_mesh(const std::string& file_path) {
  // Make sure we actually have a TSDF map/layer.
  if (!tsdf_map_pointer) {
    ROS_ERROR("[VoxbloxMap] export_mesh: tsdf_map_pointer is null. Did you call addTSDFLayer()?");
    return;
  }
  if (!tsdf_map_pointer->getTsdfLayerPtr()) {
    ROS_ERROR("[VoxbloxMap] export_mesh: TSDF layer pointer is null.");
    return;
  }

  // Get the TSDF layer from your TSDF map.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_pointer->getTsdfLayerPtr();

  // Create a mesh layer (same voxel size / voxels per side as the TSDF layer).
  auto mesh_layer = std::make_shared<voxblox::MeshLayer>(tsdf_layer->block_size());

  // Mesh integrator
  voxblox::MeshIntegratorConfig mesh_config;
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
      mesh_config, tsdf_layer , mesh_layer.get());

  // Generate mesh
  mesh_integrator.generateMesh(true, false);

  // Save to file (use the argument, not a hardcoded name)
  const bool ok = voxblox::outputMeshLayerAsPly(file_path, *mesh_layer);
  if (!ok) {
    ROS_WARN("[VoxbloxMap] export_mesh: outputMeshLayerAsPly failed (mesh empty or file error).");
  }

}


void VoxbloxMap::export_TSDF_to_file_explicit(const std::string& file_path){

  float dist = -100 ; 
  int x = 0 ;
  int y = 0 ; 
  int z = 0 ; 

  std::ofstream file(file_path);  // Opens (or creates) the file

  if (!file) {  // Check if the file opened correctly
      std::cerr << "Error opening file!" << std::endl;
      return;
  }


  voxblox::BlockIndexList blocks;
  tsdf_map_pointer->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = tsdf_map_pointer->getTsdfLayerPtr()->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = tsdf_map_pointer->getTsdfLayerPtr()->getBlockByIndex(index);

    voxblox::Point origin = block.origin();

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
          ++linear_index) {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);

      dist = voxel.distance ; 

      if ( std::abs(dist) < 2* c_voxel_size_){

        x = static_cast<int>(std::floor(coord.x() / c_voxel_size_));
        y = static_cast<int>(std::floor(coord.y() / c_voxel_size_));
        z = static_cast<int>(std::floor(coord.z() / c_voxel_size_));


        file << x << " " << y << " " << z << " " << dist << '\n' ; 

      }


    
    }
  
  }

  file.close();


}



}