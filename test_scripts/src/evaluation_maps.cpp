#include <voxblox_map/voxblox_map.h>
#include "ros/ros.h"
#include "voxblox_ros/voxblox_server.h"
#include "voxblox_ros/tsdf_server.h"
#include "voxblox/core/block.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/io/layer_io.h"
// #include <planning/modules/utils.h>


#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>



class evaluation_maps
{
private:
    voxblox_map::VoxbloxMap map;
    voxblox_map::VoxbloxMap groundtruth;

    std::string input_filepath_;
    std::string input_filepath_groundtruth_; 

    std::string input_listpath_ ; 

    float voxels_per_side_ ; 
    float tsdf_voxels_per_side_ ; 
    float voxel_size_ ; 
    float tsdf_voxel_size_ ; 
    float weight_saturation_ ;
    float truncation_dist_ ; 
    float distance_min_ ;
    float distance_max_;

    //memory
    std::vector<float> voxels_filtered_tab_ ;
    std::vector<float> map_error_tab_ ;
    std::vector<float> map_error_weighted_tab_ ;


    // first index : map number
    // second index : quality threshold
    std::vector<std::vector<float>> map_quality_tab_;
    int nb_map ; 



    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher tsdf_pointcloud_pub_;
    ros::Publisher tsdf_occupied_pointcloud_pub_;
    ros::Publisher groundtruth_occupied_pointcloud_pub_ ;

    voxblox::VoxbloxServer voxblox_server_;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_from_file;
    voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_from_file_groundtruth;
    BoundingBox BB_ ; 

public:
    evaluation_maps(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~evaluation_maps();

    float compute_voxels_in_map(int truncation_number);
    float compute_voxels_in_groundtruth();
    float compute_voxels_in_map_corrected(int truncation_number, float& square_error, float& square_weighted_error);
    float compute_voxels_in_map_corrected2(int truncation_number, float& square_error, float& square_weighted_error);
    float compute_weight_average_in_map(float truncation_dist, float weight_saturation); 
    void publish_pointcloud();
    void publish_occupied_pointcloud(voxblox_map::VoxbloxMap this_map, ros::Publisher this_pub, int this_r, int this_g, int this_b, bool bool_groundtruth);
    void evaluate_single_map(bool print, int index);

    void export_TSDF(const std::string output_filepath);
    void export_groundtruth_TSDF(const std::string output_filepath);

    void export_mesh(const std::string output_filepath);
    void export_groundtruth_mesh(const std::string output_filepath);





};

evaluation_maps::evaluation_maps(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)     : nh_(nh),
nh_private_(nh_private),
voxblox_server_(nh_, nh_private_) 
{

    //weight_saturation_ = 0.8;
    //truncation_dist_ = 0.5;
    
    // Get the voxblox map file path
    nh_private_.param("voxblox_path", input_filepath_, input_filepath_);


    //Get the groundtruth file path
    nh_private_.param("groundtruth_path", input_filepath_groundtruth_, input_filepath_groundtruth_);
    if( input_filepath_groundtruth_.empty()){
        ROS_INFO("No groundtruth for comparison received");
    }
    else{
        ROS_INFO("Received groundtruth filepath: %s", input_filepath_groundtruth_.c_str());
    }

    nh_private_.param("maplist_path", input_listpath_, input_listpath_);
    if( input_listpath_.empty()){
        ROS_INFO("No list path received");
    }
    else{
        ROS_INFO("Received list path filepath: %s", input_listpath_.c_str());
    }
    

    nh_private_.param("voxels_per_side_in_block", voxels_per_side_, voxels_per_side_ );
    nh_private_.param("tsdf_voxels_per_side", tsdf_voxels_per_side_, tsdf_voxels_per_side_ );
    nh_private_.param("voxel_size", voxel_size_, voxel_size_);
    nh_private_.param("tsdf_voxel_size", tsdf_voxel_size_, tsdf_voxel_size_);
    nh_private_.param("tsdf_voxel_size", weight_saturation_, weight_saturation_);

    ROS_INFO("Voxel size is  %f", voxel_size_);
    ROS_INFO("TSDF Voxel size is  %f", tsdf_voxel_size_);
    ROS_INFO("Voxels per side  %f", voxels_per_side_);
    ROS_INFO("TSDF Voxels per side  %f", tsdf_voxels_per_side_);

    //ROS_INFO("Weight saturation  %f", weight_saturation_);
    //ROS_INFO("Truncation dist  %f", truncation_dist_);


    map = voxblox_map::VoxbloxMap(voxel_size_, voxels_per_side_);
    if( !input_filepath_groundtruth_.empty()){
        groundtruth = voxblox_map::VoxbloxMap(voxel_size_, voxels_per_side_);
    }

    //Get Bounding Box
    //BB_ = {20.0, -25.0, 20.0, -20.0, 20.0, 0.0} ; // House
    // BB_ = {100.0, -100.0, 100.0, -100.0, 4.0, 0.0} ; // Tunnel

    std::vector<double> bounding_box_vec;
    if (nh_.hasParam("NBV_selector/bounding_box")) {
        nh_.param("NBV_selector/bounding_box", bounding_box_vec, bounding_box_vec);
        if (bounding_box_vec.size() == 6) {
            ROS_INFO("Received user bounding_box: {%f, %f, %f, %f, %f, %f} that will overwrite the default value", bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]);
            BB_ = {bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]} ;
        } else {
            ROS_WARN("Received user bounding box param from ros but with the wrong size. Please check your setup.");
        }
    } else if (nh_.hasParam("/default/bounding_box")) {
        nh_.param("/default/bounding_box", bounding_box_vec, bounding_box_vec);
        if (bounding_box_vec.size() == 6) {
            ROS_INFO("Received default bounding_box: {%f, %f, %f, %f, %f, %f}", bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]);
            BB_ =  {bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]};
        } else {
            ROS_WARN("Received default bounding box param from ros but with the wrong size. Please check your setup.");
        }
    } else {
        ROS_WARN("Did not receive any bounding box so will use NBV_selector's default.");
    }

    nh.param("/NBV_selector/distance_min", distance_min_, distance_min_);
    ROS_INFO("Distance min: %f", distance_min_ );

    nh.param("/NBV_selector/distance_max", distance_max_, distance_max_);
    ROS_INFO("Distance max: %f", distance_max_ );


    tsdf_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
            "tsdf_pointcloud", 1, true);

    tsdf_occupied_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
            "evaluation_occupied_pointcloud", 1, true);
    
    groundtruth_occupied_pointcloud_pub_ =
        nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
            "evaluation_occupied_groundtruth", 1, true);


    if( ! input_filepath_groundtruth_.empty()){
        ROS_INFO("Loading groundtruth");
        voxblox::io::LoadLayer<voxblox::TsdfVoxel>(input_filepath_groundtruth_, &layer_from_file_groundtruth); 
    }


    //publish_pointcloud();

    if( ! input_filepath_groundtruth_.empty()){
        groundtruth.addTSDFLayer( layer_from_file_groundtruth );
        publish_occupied_pointcloud(groundtruth, groundtruth_occupied_pointcloud_pub_, 255,0,0, false);

    }
    

    
    if( ! input_listpath_.empty()){
        nb_map = 0;
        int number_of_run = 5;
        nh.param("/number_of_run", number_of_run, number_of_run);
        map_quality_tab_.resize(number_of_run, std::vector<float>(10, 0.0f)); 


        for(int i= 1; i<= number_of_run ;i++){

            std::string ext = ".tsdf";
            size_t pos = input_listpath_.rfind(ext);            // find the extension
            if (pos != std::string::npos && pos > 0) {
                // character just before ".tsdf"
                size_t digitPos = pos - 1;
                input_listpath_.replace(digitPos, 1, std::to_string(i));     // replace it with what you want
            }

            ROS_INFO("Received list path filepath: %s", input_listpath_.c_str());
            evaluate_single_map(false, i);
           

        }


    }
    else{
        map_quality_tab_.resize(1, std::vector<float>(10, 0.0f)); 
        evaluate_single_map(true, 0); 

    }

    float sum = 0;
    float threshold_quality = 0 ;
    float dist_to_qual = 0 ;
    if( ! input_listpath_.empty()){

        sum = std::accumulate(voxels_filtered_tab_.begin(), voxels_filtered_tab_.end(), 0.0f);
        ROS_INFO("[RESULT] Mean corrected surface voxels in the maps are %f", sum/5.0);
        sum = std::accumulate(map_error_tab_.begin(), map_error_tab_.end(), 0.0f);
        ROS_INFO("[RESULT] Mean square error for the maps are %f", sum/5.0);
        sum = std::accumulate(map_error_weighted_tab_.begin(), map_error_weighted_tab_.end(), 0.0f);
        ROS_INFO("[RESULT] Weighted square error for the maps are %f", sum/5.0);


        for(int i = 1; i<10;i++){
            threshold_quality = (1.0/(i*i)) ;
            sum = 0 ;
            //sum = std::accumulate(map_quality_tab_[i-1].begin(), map_quality_tab_[i-1].end(), 0.0f);
            sum = map_quality_tab_[0][i-1]/voxels_filtered_tab_[0] + map_quality_tab_[1][i-1]/voxels_filtered_tab_[1] + map_quality_tab_[2][i-1]/voxels_filtered_tab_[2] + map_quality_tab_[3][i-1]/voxels_filtered_tab_[3] + map_quality_tab_[4][i-1]/voxels_filtered_tab_[4] ;
            sum = sum / 5.0 ; 
            dist_to_qual= (sum - threshold_quality)/threshold_quality * 100 ; 

            ROS_INFO("[RESULT] Saturation for distance %d is %f", i, threshold_quality);
            ROS_INFO("[RESULT] Map quality is %f", sum);
            ROS_INFO("[RESULT] Distance to quality objective is %f", dist_to_qual);


        }
    }





    
    

}

evaluation_maps::~evaluation_maps()
{
}

void evaluation_maps::evaluate_single_map(bool print, int index){
    if(print){
        ROS_INFO("Only one map to evaluate "); 
        ROS_INFO("Loading result map");
        voxblox::io::LoadLayer<voxblox::TsdfVoxel>(input_filepath_, &layer_from_file); 

    }
    else{
         voxblox::io::LoadLayer<voxblox::TsdfVoxel>(input_listpath_, &layer_from_file); 
    }
 

    map.addTSDFLayer(layer_from_file);
    publish_occupied_pointcloud(map, tsdf_occupied_pointcloud_pub_, 0,0,0, false);

    //int truncation_number = truncation_dist_ / voxel_size_ ; 
    int truncation_number = 1 ; //has to be one because groundtruth is only built on surface voxels
    truncation_dist_ = voxel_size_ ; 
    //float nb_voxels = compute_voxels_in_map(truncation_number) ; 
    float nb_voxels = 0 ; 
    float nb_voxels2 = 0 ; 
    if(print){
        //ROS_INFO("[RESULT] Surface voxels in the map are %f", nb_voxels);
    }

    if( ! input_filepath_groundtruth_.empty()){

        nb_voxels = compute_voxels_in_groundtruth() ; 
        if(print){
            ROS_INFO("[RESULT] Surface voxels in groundtruth %f", nb_voxels);
        }
        float square_error = 0 ; 
        float square_weighted_error = 0 ; 
        nb_voxels = compute_voxels_in_map_corrected(truncation_number, square_error, square_weighted_error) ; 
        nb_voxels2 = compute_voxels_in_map_corrected2(truncation_number, square_error, square_weighted_error) ; 
        export_mesh("/root/.ros/log/tsdfs/map_local.ply");
        export_groundtruth_mesh("/root/.ros/log/tsdfs/groundtruth_tsdf.ply") ;
        
        ROS_INFO("[RESULT] Corrected surface voxels in the map are %f", nb_voxels);
        ROS_INFO("[RESULT] Square error for the map is %f", square_error);
        
        voxels_filtered_tab_.push_back(nb_voxels);
        map_error_tab_.push_back(square_error);
        map_error_weighted_tab_.push_back(square_weighted_error);

    }



    float quality = 0 ;
    float threshold_quality = -1;

    //float saturation_dist_min = 1.0/( distance_min_ * distance_min_);
    //quality = compute_weight_average_in_map(truncation_dist_, saturation_dist_min);//CAREFUL TRUNCATION DIST IN METERS !!! 
    if(print){
        //ROS_INFO("[RESULT] Saturation for distance min %f", distance_min_);
        //ROS_INFO("[RESULT] Map quality is %f", quality);
    }


    //float saturation_dist_max = 1.0/( distance_max_ * distance_max_);
    //quality = compute_weight_average_in_map(truncation_dist_, saturation_dist_max) ; //CAREFUL TRUNCATION DIST IN METERS !!! 
    if(print){
        //ROS_INFO("[RESULT] Saturation for distance max %f", distance_max_);
        //ROS_INFO("[RESULT] Map quality is %f", quality);
    }



    for(int i = 1; i<10;i++){
        threshold_quality = (1.0/(i*i)) ;
        quality = compute_weight_average_in_map(truncation_dist_, threshold_quality) ; //CAREFUL TRUNCATION DIST IN METERS !!! 
        if(print){

            ROS_INFO("[RESULT] Saturation for distance %d is %f", i, threshold_quality);
            ROS_INFO("[RESULT] Map quality is %f", quality);

        }
        map_quality_tab_[index-1][i-1] = quality ; 




    }



}

void evaluation_maps::publish_pointcloud(){
    if (voxblox::io::LoadLayer<voxblox::TsdfVoxel>(input_filepath_, &layer_from_file)) {
        ROS_INFO("Could load TSDF map!");
        pcl::PointCloud<pcl::PointXYZI> pointcloud;

        createDistancePointcloudFromTsdfLayer(*layer_from_file, &pointcloud);

        pointcloud.header.frame_id = "world";
        tsdf_pointcloud_pub_.publish(pointcloud);
    } else {
        ROS_ERROR("Couldn't load TSDF map!");
    }

}

void evaluation_maps::publish_occupied_pointcloud(voxblox_map::VoxbloxMap this_map, ros::Publisher this_pub, int this_r, int this_g, int this_b, bool bool_groundtruth){

    pcl::PointCloud<pcl::PointXYZRGB> occupied_pointcloud;
    unsigned char current_state;
    voxblox::BlockIndexList blocks;
    this_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);
    bool isInBB;
    int pt_size = 0 ; 



    // Cache layer settings.
    size_t vps = this_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    //ROS_INFO("a %f", vps);


    for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = this_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->getBlockByIndex(index);

      for (size_t linear_index = 0; linear_index < num_voxels_per_block;
          ++linear_index) {
        voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
        const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
        
        Eigen::Vector3d coord_3d = Eigen::Vector3d(coord.x(), coord.y(), coord.z());
        //ROS_INFO("b %f %f %f", coord.x(), coord.y(), coord.z());



        current_state = this_map.getVoxelState_TSDF(coord_3d, 0.0); //CAREFUL : 2ND PARAMETER SHOULD BE ALIGNED WITH THRESHOLD KNOWN
        
        isInBB = isCorrectPos(coord_3d, BB_);
        if ( (current_state == voxblox_map::VoxbloxMap::OCCUPIED) && isInBB ){

            pcl::PointXYZRGB point;

            if( bool_groundtruth ){
                point.x = -coord.y();
                point.y = coord.x();
                point.z = coord.z();

            }
            else{
                point.x = coord.x();
                point.y = coord.y();
                point.z = coord.z();

                // if ( coord.z() <= 0.25 ){
                //     ROS_INFO(" Ground value is %f", this_map.getVoxelDistance_TSDF(coord_3d));
                // }

            }
            point.r = this_r;
            point.g = this_g;
            point.b = this_b;
            occupied_pointcloud.push_back(point);
            pt_size = pt_size +1 ; 
        }

      }
    }

    occupied_pointcloud.header.frame_id = "world";
    this_pub.publish(occupied_pointcloud);
    ROS_INFO("Published pointcloud of size %d", pt_size);




}

float evaluation_maps::compute_voxels_in_map(int truncation_number){
    return map.countvoxelsinmap_TSDF(BB_, truncation_number) ; 

}

float evaluation_maps::compute_voxels_in_groundtruth(){
    return groundtruth.countvoxelsinmap_TSDF(BB_, 1) ; 

}

float evaluation_maps::compute_voxels_in_map_corrected(int truncation_number, float& square_error, float& square_weighted_error){
    float voxel_count_covered = 0 ; 
    std::unordered_map<std::string, float> result_map ; 
    voxel_count_covered = map.countvoxelsinmap_TSDF_corrected(BB_, truncation_number, groundtruth, result_map) ; 
    square_error = result_map["error"];  
    square_weighted_error = result_map["weighted error"] ;
    //ROS_INFO(" Groundtruth surface voxels only in the covered area are %f ", result_map["voxel surface groundtruth"]) ; 
    //ROS_INFO(" Perfect zeros %f ", result_map["perfect_zeros"]) ; 
    return voxel_count_covered ; 


}

float evaluation_maps::compute_voxels_in_map_corrected2(int truncation_number, float& square_error, float& square_weighted_error){
    float voxel_count_covered = 0 ; 
    std::unordered_map<std::string, float> result_map ; 
    voxel_count_covered = map.count_surface_voxels(BB_, truncation_number, groundtruth, result_map) ; 
    square_error = result_map["error"];  
    square_weighted_error = result_map["weighted error"] ;
    //ROS_INFO(" Groundtruth surface voxels only in the covered area are %f ", result_map["voxel surface groundtruth"]) ; 
    //ROS_INFO(" Perfect zeros %f ", result_map["perfect_zeros"]) ; 
    return voxel_count_covered ; 


}

//Z_sat
float evaluation_maps::compute_weight_average_in_map(float truncation_dist, float weight_saturation){
    float maxx = -1 ;
    float minn = -1 ;
    int nb_voxels;
    float result = -1 ;
    if( ! input_filepath_groundtruth_.empty()){
        ROS_INFO("Using groundtruth ");
        result = map.evaluation_TSDF(truncation_dist, maxx, minn, nb_voxels, BB_, weight_saturation, groundtruth) ; 
    }
    else{
        ROS_INFO("Not using groundtruth ");
        float result =  map.evaluation_TSDF(truncation_dist, maxx, minn, nb_voxels, BB_, weight_saturation) ; 
    }


    //ROS_INFO("Max weight is %f ", maxx);
    //ROS_INFO("Min weight is %f ", minn);
    ROS_INFO("Nb voxels analyzed is %d ", nb_voxels);

    //ROS_INFO("Voxel size is %f ", map.getVoxelSize());



    return result;
}

void evaluation_maps::export_TSDF(const std::string output_filepath){
    map.export_TSDF_to_file_explicit(output_filepath);
}

void evaluation_maps::export_groundtruth_TSDF(const std::string output_filepath){
    groundtruth.export_TSDF_to_file_explicit(output_filepath);
}


void evaluation_maps::export_mesh(const std::string output_filepath){
    map.export_mesh(output_filepath);
}

void evaluation_maps::export_groundtruth_mesh(const std::string output_filepath){
    groundtruth.export_mesh(output_filepath);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "exploration_comparator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");  

    evaluation_maps evaluation_maps(nh, nh_private);
    ros::spin();
    return 0;    
}