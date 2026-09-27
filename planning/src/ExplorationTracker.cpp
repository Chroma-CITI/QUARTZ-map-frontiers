#include "ros/ros.h"
#include "voxblox_ros/voxblox_server.h"
#include "voxblox_ros/tsdf_server.h"
#include "voxblox/core/block.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/io/layer_io.h"

#include <voxblox_map/utils.h>
#include <voxblox_map/voxblox_map.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>
#include <iostream>

class ExplorationTracker {
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    // Map!
    voxblox::VoxbloxServer voxblox_server_;
    /// Publish the complete map for other nodes to consume.
    ros::Subscriber tsdf_subscriber_;
    BoundingBox bounding_box_ = {100.0, -100.0, 100.0, -100.0, 50.0, 0.0}; // {xmax, xmin, ymax, ymin, zmax, zmin}
    
    voxblox_map::VoxbloxMap map_;
    voxblox_map::VoxbloxMap groundtruth_map_;

    ros::Time initial_time_;
    ros::Time last_map_timestamp_;
    ros::Duration min_duration_since_last_computation_;

    // Log file
    std::string logfilename_;
    // Groundtruth filepath
    std::string input_filepath_groundtruth_;
    
    float distance_min;
    float distance_max;
    float weight_min_ ; 

    void GetRosParams() {
        //Get the groundtruth file path
        nh_private_.param("groundtruth_path", input_filepath_groundtruth_, input_filepath_groundtruth_);
        std::ifstream file_groundtruth(input_filepath_groundtruth_.c_str());
        if( input_filepath_groundtruth_.empty()){
            ROS_ERROR("[ExplorationComparator] No groundtruth for comparison received");
        }
        else if (!file_groundtruth.good()){
            ROS_ERROR("[ExplorationComparator] The received groundtruth file does not exist! Add it in %s", input_filepath_groundtruth_.c_str());
        } else {
            ROS_INFO("[ExplorationComparator] Received groundtruth filepath: %s", input_filepath_groundtruth_.c_str());
        }

        std::vector<double> bounding_box_vec;
        if (nh_.hasParam("NBV_selector/bounding_box")) {
            nh_.param("NBV_selector/bounding_box", bounding_box_vec, bounding_box_vec);
            if (bounding_box_vec.size() == 6) {
                ROS_INFO("[ExplorationTracker] Received user bounding_box: {%f, %f, %f, %f, %f, %f} that will overwrite the default value", bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]);
                bounding_box_ = {bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]} ;
            } else {
                ROS_WARN("[ExplorationTracker] Received user bounding box param from ros but with the wrong size. Please check your setup.");
            }
        } else if (nh_.hasParam("/default/bounding_box")) {
            nh_.param("/default/bounding_box", bounding_box_vec, bounding_box_vec);
            if (bounding_box_vec.size() == 6) {
                ROS_INFO("[ExplorationTracker] Received default bounding_box: {%f, %f, %f, %f, %f, %f}", bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]);
                bounding_box_ =  {bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]};
            } else {
                ROS_WARN("[ExplorationTracker] Received default bounding box param from ros but with the wrong size. Please check your setup.");
            }
        } else {
            ROS_WARN("[ExplorationTracker] Did not receive any bounding box so will use NBV_selector's default.");
        }

        // Get the logfile name
        std::string log_dir = "/root/.ros/log/maps/";
        nh_private_.param("log_dir", log_dir, log_dir);
        std::string log_file = "explored_voxels_overtime.csv";
        nh_private_.param("logfilename", log_file, log_file);
        logfilename_ = log_dir + log_file;
        ROS_INFO("should save logs to file %s",logfilename_.c_str());

        // Get the voxblox map file path
        float voxel_size; 
        float voxels_per_side;
        nh_private_.param("voxels_per_side", voxels_per_side, voxels_per_side);
        nh_private_.param("voxel_size", voxel_size, voxel_size);
        ROS_INFO("[ExplorationTracker] Voxel size is  %f", voxel_size);
        ROS_INFO("[ExplorationTracker] Voxels per side  %f", voxels_per_side);
        map_ = voxblox_map::VoxbloxMap(voxel_size, voxels_per_side);

        //Get the objective weight
        nh_private_.param("/NBV_selector/distance_min", distance_min, distance_min);
        ROS_WARN("Exploration Tracker got Distance min: %f", distance_min );

        nh_private_.param("/NBV_selector/distance_max", distance_max, distance_max);
        ROS_WARN("Exploration Tracker got Distance max: %f", distance_max );
        weight_min_ = 1.0 / (distance_max * distance_max) ; 
        ROS_WARN("Exploration Tracker weight min: %f", weight_min_ );


        // Get the ground truth map file
        groundtruth_map_ = voxblox_map::VoxbloxMap(voxel_size, voxels_per_side);
        voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_from_file_groundtruth;
        voxblox::io::LoadLayer<voxblox::TsdfVoxel>(input_filepath_groundtruth_, &layer_from_file_groundtruth);
        groundtruth_map_.addTSDFLayer(layer_from_file_groundtruth);

        // Get the minimum time since last computation
        min_duration_since_last_computation_ = ros::Duration(10); // In seconds
        float min_time_since_last_computation;
        if (nh_.hasParam("min_time_since_last_computation")) {
            nh_private_.param("min_time_since_last_computation", min_time_since_last_computation, min_time_since_last_computation);
            min_duration_since_last_computation_ = ros::Duration(min_time_since_last_computation); // In seconds
        }

        initial_time_ = ros::Time::now();
        last_map_timestamp_ = ros::Time::now();
        tsdf_subscriber_ = nh_.subscribe("tsdf_map_in", 1, &ExplorationTracker::tSDFCallback, this);

        // Write header if logfile is empty
        std::ofstream file(logfilename_, std::ios_base::app);
        if (file.tellp() == 0) {
            file << "time_since_start,quality\n";
        }
        file.close();
    }

    void tSDFCallback(const voxblox_msgs::Layer& layer_msg){
        ROS_DEBUG("[ExplorationTracker] tSDFCallback");
        bool success =
            voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(layer_msg, map_.get_tsdf_map_pointer()->getTsdfLayerPtr());

        if (!success) {
            ROS_ERROR_THROTTLE(10, "[ExplorationTracker] Got an invalid TSDF map message!");
            LOG(ERROR) << "layer_msg voxel size = " << layer_msg.voxel_size << " map layer voxel size = " << map_.get_tsdf_map_pointer()->getTsdfLayerPtr()->voxel_size();
            LOG(ERROR) << "layer_msg voxel per side = " << layer_msg.voxels_per_side << " map layer voxel per side = " << map_.get_tsdf_map_pointer()->getTsdfLayerPtr()->voxels_per_side();
            return;
        }
        // Checking if it has been more than min_duration_since_last_computation_ passed before computing again
        ros::Duration time_since_last_computation = ros::Time::now() - last_map_timestamp_;
        if (time_since_last_computation.toSec() < min_duration_since_last_computation_.toSec()) {
            ROS_DEBUG("[ExplorationTracker] Not enough time has elapsed (%f s < %f s) since last compuation.", time_since_last_computation.toSec(), min_duration_since_last_computation_.toSec());
            return;
        }
        last_map_timestamp_ = ros::Time::now();
        ros::Duration time_since_initialization = last_map_timestamp_ - initial_time_;

        // Computing the number of explored voxels
        ros::Time start_computation = ros::Time::now();
        float nb_voxels = ComputeNumberOfExploredVoxels();
        ros::Time end_computation = ros::Time::now();
        ros::Duration computation_time = end_computation - start_computation;
        ROS_INFO("[ExplorationTracker] Surface voxels in the map are %f at time %f. Took %f s", nb_voxels, last_map_timestamp_.toSec(), computation_time.toSec());

        // Write result to csv
        std::ofstream file;
        file.open(logfilename_, std::ios_base::app); // Open in append mode
        if (file.is_open()) {
            file << time_since_initialization.toSec() << "," << nb_voxels << "\n";
            file.close();
        } else {
            std::cerr << "[ExplorationTracker] Unable to open file: " << logfilename_ << std::endl;
        }
    }

    float ComputeNumberOfExploredVoxels() {
        // Log explored voxels during exploration
        
        // float truncation_dist = 0.5; //CAREFUL TRUNCATION DIST IN METERS !!!
        // float truncation_number = truncation_dist /  map_.getVoxelSize() ; 
        // float weight_saturation = 1 ; 

        // float nb_voxels = map_.countvoxelsinmap_TSDF(bounding_box_, truncation_number); 

        float square_error = 0;
        float square_weighted_error = 0;
        std::unordered_map<std::string, float> map_result ;  
        int truncation_number = 1 ; //has to be one because groundtruth is only built on surface voxels
        //float nb_voxels = map_.countvoxelsinmap_TSDF_corrected(bounding_box_, truncation_number, groundtruth_map_, map_result);
        float nb_voxels = map_.count_surface_voxels_no_GD(bounding_box_, truncation_number, weight_min_);
        return nb_voxels;
    }


public:
    ExplorationTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) 
    : nh_(nh),
      nh_private_(nh_private),
      voxblox_server_(nh_, nh_private_) 
    {
        // Get all rosparams
        GetRosParams();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_progress_tracker");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");  

    ExplorationTracker explorationTracker(nh, nh_private);
    ros::spin();
    return 0;
}