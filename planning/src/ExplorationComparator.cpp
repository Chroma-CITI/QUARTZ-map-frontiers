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

class ExplorationComparator {
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    // Map!
    voxblox::VoxbloxServer voxblox_server_;
    /// Publish the complete map for other nodes to consume.
    ros::Publisher tsdf_pointcloud_pub_;
    BoundingBox bounding_box_ = {100.0, -100.0, 100.0, -100.0, 50.0, 0.0}; // {xmax, xmin, ymax, ymin, zmax, zmin}

    std::string input_filepath_;
    // Groundtruth filepath
    std::string input_filepath_groundtruth_; 

    voxblox_map::VoxbloxMap map_;
    voxblox_map::VoxbloxMap groundtruth_map_;

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

        // Get the bounding box
        std::vector<double> bounding_box_vec;
        if (nh_.hasParam("NBV_selector/bounding_box")) {
            nh_.param("NBV_selector/bounding_box", bounding_box_vec, bounding_box_vec);
            if (bounding_box_vec.size() == 6) {
                ROS_INFO("Received user bounding_box: {%f, %f, %f, %f, %f, %f} that will overwrite the default value", bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]);
                bounding_box_ = {bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]} ;
            } else {
                ROS_WARN("Received user bounding box param from ros but with the wrong size. Please check your setup.");
            }
        } else if (nh_.hasParam("/default/bounding_box")) {
            nh_.param("/default/bounding_box", bounding_box_vec, bounding_box_vec);
            if (bounding_box_vec.size() == 6) {
                ROS_INFO("Received default bounding_box: {%f, %f, %f, %f, %f, %f}", bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]);
                bounding_box_ =  {bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]};
            } else {
                ROS_WARN("Received default bounding box param from ros but with the wrong size. Please check your setup.");
            }
        } else {
            ROS_WARN("Did not receive any bounding box so will use NBV_selector's default.");
        }

        // Get the voxel size and the voxels per side
        float voxel_size; 
        float voxels_per_side;
        nh_private_.param("voxblox_path", input_filepath_, input_filepath_);
        nh_private_.param("voxels_per_side", voxels_per_side, voxels_per_side );
        nh_private_.param("voxel_size", voxel_size, voxel_size);
        ROS_INFO("Voxel size is  %f", voxel_size);
        ROS_INFO("Voxels per side  %f", voxels_per_side);

        // Get the voxblox map file path
        map_ = voxblox_map::VoxbloxMap(voxel_size, voxels_per_side);

        // Get the ground truth map file
        groundtruth_map_ = voxblox_map::VoxbloxMap(voxel_size, voxels_per_side);
        voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_from_file_groundtruth;
        voxblox::io::LoadLayer<voxblox::TsdfVoxel>(input_filepath_groundtruth_, &layer_from_file_groundtruth);
        groundtruth_map_.addTSDFLayer(layer_from_file_groundtruth);
        }

    void ComputeAndLogResults() {
        // Log explored voxels during exploration
        std::unordered_map<std::string, float> map_result;
        std::unordered_map<std::string, float> map_result2;
        float nb_voxels = ComputeNumberOfExploredVoxels(map_result);
        float nb_voxels2 = ComputeNumberOfExploredVoxels2(map_result2);
        ROS_WARN("[ExplorationComparator] Surface voxels in the map are %f", nb_voxels);
        ROS_WARN("[ExplorationComparator] Surface voxels 2  in the map are %f", nb_voxels2);

        // Get the logfile name
        std::string log_dir = "/root/.ros/log/maps/";
        nh_private_.param("log_dir", log_dir, log_dir);
        std::string log_file = "explored_voxels.txt";
        nh_private_.param("logfilename", log_file, log_file);
        std::string log_file_path = log_dir + log_file;
        std::string mesh_file = "mesh.ply";
        std::string mesh_file_path = log_dir + mesh_file ; 
        map_.export_mesh(mesh_file_path); 

        std::ofstream out(log_file_path, std::ios::out | std::ios::app);  // append mode
        if (out.is_open()) {
            out << "Surface voxels in the map: " << nb_voxels << "\n";
            out << "Map error: " << map_result["error"] << "\n" ; 
            out << "Weighted map error: " << map_result["weighted error"] << "\n" ; 
            out << "Voxels too far: " << map_result["voxels too far"] << "\n" ; 

            out << "Map error 2: " << map_result2["error"] << "\n" ; 
            out << "Surface voxels 2: " << nb_voxels2 << "\n" ; 
            out << "Groundtruth surface voxels 2:" << map_result2["groundtruth surface"] << "\n" ; 

            out.close();
        } else {
            ROS_WARN("[ExplorationComparator] Failed to open file for destructor log.");
        }
    }

    float ComputeNumberOfExploredVoxels(std::unordered_map<std::string, float>& map_result ) {
        float square_error = 0;
        float square_weighted_error = 0; 
        int truncation_number = 1 ; //has to be one because groundtruth is only built on surface voxels
        float nb_voxels = map_.countvoxelsinmap_TSDF_corrected(bounding_box_, truncation_number, groundtruth_map_, map_result);
        return nb_voxels;
    }

    float ComputeNumberOfExploredVoxels2(std::unordered_map<std::string, float>& map_result ) {
        float square_error = 0;
        float square_weighted_error = 0; 
        int truncation_number = 1 ; //has to be one because groundtruth is only built on surface voxels
        float nb_voxels = map_.count_surface_voxels(bounding_box_, truncation_number, groundtruth_map_, map_result);
        return nb_voxels;
    }


public:
    ExplorationComparator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) 
    : nh_(nh),
      nh_private_(nh_private),
      voxblox_server_(nh_, nh_private_) 
    {
        // Get the voxblox map file path
        nh_private_.param("voxblox_path", input_filepath_, input_filepath_);

        // Get the bounding box
        GetRosParams();

        // Try to read the tsdf file
        voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_from_file;
        if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>(input_filepath_, &layer_from_file)) {
            ROS_INFO("Could load TSDF map!");
            // Create a map from the tsdf layer
            map_.addTSDFLayer(layer_from_file);
            ROS_INFO("Could add layer to map!");

            // Compute the number of explored voxels and log the result
            ComputeAndLogResults();

        } else {
            ROS_ERROR("Couldn't load TSDF map!");
        }
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration_comparator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");  

    ExplorationComparator explorationComparator(nh, nh_private);
    return 0;
}