#pragma once

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <vector>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include <voxblox_map/utils.h>

class NBVSelectorParameters {
private:
    std::unordered_set<std::string> _views_generation_methods = {"gradient", "sphere"};

protected:
    void CheckValues();
    void SettingProfileMethods(const std::string& profile);

public:
    /////////////// GENERAL
    std::string profile; 
    float quality_objective;  // Coefficient between 0 and 1. TODO: description!
    bool use_freespace;

    /////////////// NAVIGATION
    float tolerance_distance; // In m. Max acceptable distance between current pose and goal for it to be seen as reached
    float angular_tolerance;  // In degrees. Max acceptable angle between current pose and goal for it to be seen as reached
    double threshold_known;  // from Hardouin // NOTE: Is it used?

    ////////////// Planning parameters
    float image_component_weight; // TODO: description
    float metrics_component_weight; // TODO: description
    float topology_component_weight; // TODO: description
    float coordination_component_weight;
    bool use_distance_in_metrics_component;  // TODO: description
    bool do_occlusions_check; // TODO: description
    bool forced_safety; // Force views candidate to be in the empty freespace
    bool remove_inacess_frontiers ;
    bool use_topology ; 
    bool use_coordination ;
    bool use_replanning ; 
    bool use_repulsion_max ; 
    bool use_repass ; 

    ////////////// View Generation parameters
    std::string views_generation_method; // TODO: description
    float distance_min; // In m. Min distance between the frontier point and the generated view.
    float distance_max; // In m. Max distance between the frontier point and the generated view.
    int subsampling_views; // Number of subsampled views. The bigger the faster but the less likely to find a NBV.
    float robot_radius; // Max number of voxels occupied by the robots in one direction : if 5, robot is contained in a 5*5*5 voxel cube
    int radius_surface_max; // In m. zone around the frontier to search for surface voxels
    float angle_low; // In degrees. Vertical angle below the drone 
    float angle_high; // In degrees. Vertical angle above the drone 

    ////////////// View Evaluator parameters
    float value_frontier;  // NOTE: Is it used?
    std::string views_selection_method; 

    ////////////// View Evaluator parameters
    std::string views_sampling_method; 

    //Coordination parameters
    std::string dispersion_method; 

    ////////////// LIDAR Parameters
    // TODO: should be only in SensorModel
    Eigen::Vector3d mounting_translation_;  // x,y,z [m] // NOTE: Is it used?
    Eigen::Quaterniond mounting_rotation_;  // x,y,z,w quaternion  // NOTE: Is it used?
    //sensor parameters
    double p_ray_length;  // In m. Expected max sensor ray length
    double p_fov_x;  // In degrees. Total fields of view, expected symmetric w.r.t.
    // sensor facing direction
    double p_fov_y;  // In degrees. Total fields of view, expected symmetric w.r.t.
    int p_resolution_x; // In pixel. Expected resolution of the sensor
    int p_resolution_y; // In pixel. Expected resolution of the sensor
    double p_sampling_time; // In s. 


    ////////////// Bounding Box
    // In m. Max/min value that will define an exploration bounding box in which the views
    // and next best views can be computed
    BoundingBox bounding_box; // {xmax, xmin, ymax, ymin, zmax, zmin}

    ////////////// QUality Areas
    // List of areas defined by a bounding box for which a certain quality is applied
    std::vector<QualityArea> quality_areas_;

    ////////////// Logs
    bool verbose;
    bool verbose_planning;
    bool verbose_multi;
    bool verbose_view_evaluation;
    bool verbose_view_generation;
    bool timer;
    bool extra_viz;

    // Constructor
    NBVSelectorParameters();
    NBVSelectorParameters(const std::string& profile);

    void SetDefaultValues();

    void LoadFromRos(const ros::NodeHandle& nh);
};


NBVSelectorParameters::NBVSelectorParameters(){
    SetDefaultValues();
}

NBVSelectorParameters::NBVSelectorParameters(const std::string& profile){
    SetDefaultValues();
    SettingProfileMethods(profile);
}

void NBVSelectorParameters::SetDefaultValues(){
    quality_objective = 1.0;
    use_freespace = false;

    tolerance_distance = 0.2;
    angular_tolerance = 10.0 * M_PI / 180.0;
    threshold_known = 0.0;

    image_component_weight = 1.0;
    metrics_component_weight = 1.0;
    topology_component_weight = 0;
    coordination_component_weight = 0;
    use_distance_in_metrics_component = true; 
    do_occlusions_check = false;
    forced_safety = false; 
    remove_inacess_frontiers = false; 
    use_topology = false;
    use_coordination = false; 
    use_replanning = false ; 
    use_repulsion_max = false ; 
    use_repass = false ; 

    views_generation_method = "gradient";
    views_selection_method = "ours";
    dispersion_method="";
    distance_min = 3.0;
    distance_max = 4.0;
    subsampling_views = 100;
    robot_radius = 0.5; //m doit etre multiple de la voxel size
    radius_surface_max = 3;
    angle_low = -25.0;
    angle_high = 57.0;

    mounting_translation_ = Eigen::Vector3d();
    mounting_rotation_ = Eigen::Quaterniond();
    p_ray_length = 10.0;
    p_fov_x = 360.0 * M_PI / 180.0;
    p_fov_y = 63.05 * M_PI / 180.0;
    p_resolution_x = 1000;
    p_resolution_y = 1000;
    p_sampling_time = 1.0;

    // current value is for HOUSE env. 
    bounding_box = {100.0, -100.0, 100.0, -100.0, 50.0, 0.0};
}

void NBVSelectorParameters::LoadFromRos(const ros::NodeHandle& nh){
    nh.param("/NBV_selector/sub_sample_size", subsampling_views, subsampling_views);
    ROS_INFO("Received sub_sample_size: %i", subsampling_views);

    nh.param("/NBV_selector/angle_low", angle_low, angle_low);
    ROS_INFO("Received angle_low: %f", angle_low);

    nh.param("/NBV_selector/angle_high", angle_high, angle_high);
    ROS_INFO("Received angle_high: %f", angle_high);

    nh.param("/NBV_selector/p_ray_length", p_ray_length, p_ray_length);
    ROS_INFO("Received p_ray_length: %f", p_ray_length);

    nh.param("/NBV_selector/quality_objective", quality_objective, quality_objective);
    ROS_INFO("Received image qualitive objective: %f", quality_objective);

    nh.param("/NBV_selector/image_component_weight", image_component_weight, image_component_weight);
    ROS_INFO("Received image component coefficient: %f", image_component_weight);

    nh.param("/NBV_selector/metrics_component_weight", metrics_component_weight, metrics_component_weight);
    ROS_INFO("Received navigation component coefficient: %f", metrics_component_weight);

    nh.param("/NBV_selector/topology_component_weight", topology_component_weight, topology_component_weight);
    ROS_INFO("Received topology component coefficient: %f", topology_component_weight);

    nh.param("/NBV_selector/coordination_component_weight", coordination_component_weight, coordination_component_weight);
    ROS_INFO("Received coordination component coefficient: %f", coordination_component_weight);

    nh.param("/NBV_selector/use_distance_in_metrics_component", use_distance_in_metrics_component, use_distance_in_metrics_component);
    if (use_distance_in_metrics_component) {
      ROS_INFO("Using angular and linear distances to compute the cost weight of each view.");
    } else {
      ROS_INFO("Using angular distance only to compute the cost weight of each view.");
    }

    nh.param("/NBV_selector/do_occlusions_check", do_occlusions_check, do_occlusions_check);
    ROS_INFO("Doing occlusion check: %s", do_occlusions_check ? "true" : "false");

    nh.param("/NBV_selector/distance_min", distance_min, distance_min);
    ROS_INFO("Distance min: %f", distance_min );

    nh.param("/NBV_selector/distance_max", distance_max, distance_max);
    ROS_INFO("Distance max: %f", distance_max );

    nh.param("/NBV_selector/forced_safety", forced_safety, forced_safety);
    ROS_INFO("Forcing views in the freespace: %s", forced_safety ? "true" : "false");

    nh.param("/NBV_selector/remove_inacess_frontiers", remove_inacess_frontiers, remove_inacess_frontiers);
    ROS_INFO("Remove inacessible frontiers: %s", remove_inacess_frontiers ? "true" : "false");

    nh.param("/NBV_selector/use_topology", use_topology, use_topology);
    ROS_INFO("Using topology: %s", use_topology ? "true" : "false");

    nh.param("/NBV_selector/use_coordination", use_coordination, use_coordination);
    ROS_INFO("Using coordination: %s", use_coordination ? "true" : "false");

    nh.param("/NBV_selector/use_replanning", use_replanning, use_replanning);
    ROS_INFO("Using replanning: %s", use_replanning ? "true" : "false");

    nh.param("/NBV_selector/use_repulsion_max", use_repulsion_max, use_repulsion_max);
    ROS_INFO("Using repulsion max: %s", use_repulsion_max ? "true" : "false");

    nh.param("/NBV_selector/use_repass", use_repass, use_repass);
    ROS_INFO("Using repass: %s", use_repass ? "true" : "false");

    nh.param("/NBV_selector/use_freespace", use_freespace, use_freespace);
    ROS_INFO("Using freespace: %s", use_freespace ? "true" : "false");

    std::vector<double> bounding_box_vec;
    if (nh.hasParam("/NBV_selector/bounding_box")) {
        nh.param("/NBV_selector/bounding_box", bounding_box_vec, bounding_box_vec);
        if (bounding_box_vec.size() == 6) {
            ROS_INFO("Received user bounding_box: {%f, %f, %f, %f, %f, %f} that will overwrite the default value", bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]);
            bounding_box = {bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]} ;
        } else {
            ROS_WARN("Received user bounding box param from ros but with the wrong size. Please check your setup.");
        }
    } else if (nh.hasParam("/default/bounding_box")) {
        nh.param("/default/bounding_box", bounding_box_vec, bounding_box_vec);
        if (bounding_box_vec.size() == 6) {
            ROS_INFO("Received default bounding_box: {%f, %f, %f, %f, %f, %f}", bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]);
            bounding_box =  {bounding_box_vec[0],bounding_box_vec[1],bounding_box_vec[2],bounding_box_vec[3],bounding_box_vec[4],bounding_box_vec[5]};
        } else {
            ROS_WARN("Received default bounding box param from ros but with the wrong size. Please check your setup.");
        }
    } else {
        ROS_WARN("Did not receive any bounding box so will use NBV_selector's default.");
    }

    if (nh.hasParam("/NBV_selector/quality_areas")) {
        XmlRpc::XmlRpcValue quality_areas_list;
        if (nh.getParam("/NBV_selector/quality_areas", quality_areas_list)) {
            if (quality_areas_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                ROS_ERROR("Param quality_areas_list is not a list");
            }
        }
        // Iterate over each element in the list
        for (int i = 0; i < quality_areas_list.size(); ++i)
        {
            QualityArea quality_area;
            // Each element should be a struct with two elements: "bounding_box" and "quality"
            if (quality_areas_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_WARN("Element %d is not a struct", i);
                continue;
            }
            // Extract the bounding box (first element)
            XmlRpc::XmlRpcValue sublist = quality_areas_list[i]["bounding_box"];
            if (sublist.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_WARN("Element %d's first element is not a list", i);
                continue;
            }
            std::vector<double> quality_bounding_box_vec;
            for (int i = 0; i < sublist.size(); ++i) {
                if (sublist[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    quality_bounding_box_vec.push_back(static_cast<double>(sublist[i]));
                } else if (sublist[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                    quality_bounding_box_vec.push_back(static_cast<int>(sublist[i]));
                }
            }
            if (quality_bounding_box_vec.size() == 6) {
                quality_area.bounding_box =  {quality_bounding_box_vec[0],quality_bounding_box_vec[1],quality_bounding_box_vec[2],quality_bounding_box_vec[3],quality_bounding_box_vec[4],quality_bounding_box_vec[5]};
            } else {
                ROS_WARN("Element %d's first element could not be filled up. The type of its values should be int or float.", i);
            }

            // Extract the quality (second element)
            if (quality_areas_list[i]["quality"].getType() != XmlRpc::XmlRpcValue::TypeDouble){
                ROS_WARN("Element %d's second element is not a float", i);
                continue;
            }

            quality_area.quality = static_cast<double>(quality_areas_list[i]["quality"]);
            quality_areas_.push_back(quality_area);
            ROS_INFO("Adding the following quality area: {%f, %f, %f, %f, %f, %f} of quality %f", quality_bounding_box_vec[0],quality_bounding_box_vec[1],quality_bounding_box_vec[2],quality_bounding_box_vec[3],quality_bounding_box_vec[4],quality_bounding_box_vec[5], quality_area.quality);
        }
    }
    nh.param("/NBV_selector/views_generation_method", views_generation_method, views_generation_method);
    ROS_INFO("Received views_generation_method: %s", views_generation_method.c_str());

    nh.param("/NBV_selector/dispersion_method", dispersion_method, dispersion_method);
    ROS_INFO("Received dispersion_method: %s", dispersion_method.c_str());

    nh.param("/NBV_selector/timer", timer, timer);
    ROS_INFO("Enabling timer: %s", timer ? "true" : "false");

    nh.param("/NBV_selector/verbose", verbose, verbose);
    ROS_INFO("Enabling verbose: %s", verbose ? "true" : "false");

    nh.param("/NBV_selector/verbose_planning", verbose_planning, verbose_planning);
    ROS_INFO("Enabling verbose_planning: %s", verbose_planning ? "true" : "false");

    nh.param("/NBV_selector/verbose_multi", verbose_multi, verbose_multi);
    ROS_INFO("Enabling verbose_multi: %s", verbose_multi ? "true" : "false");

    nh.param("/NBV_selector/verbose_view_generation", verbose_view_generation, verbose_view_generation);
    ROS_INFO("Enabling verbose_view_generation: %s", verbose_view_generation ? "true" : "false");

    nh.param("/NBV_selector/verbose_view_evaluation", verbose_view_evaluation, verbose_view_evaluation);
    ROS_INFO("Enabling verbose_view_evaluation: %s", verbose_view_evaluation ? "true" : "false");

    nh.param("/NBV_selector/extra_viz", extra_viz, extra_viz);
    ROS_INFO("Enabling extra_viz: %s", extra_viz ? "true" : "false");

    // Checking profile at the end because it would overload previously set values
    const char* env_profile = std::getenv("NBV_SELECTOR_PROFILE");
    if (env_profile && *env_profile != '\0' && std::string(env_profile) != "") {
        profile = std::string(env_profile);
        ROS_INFO("Received nbv_selector_profile as a global variable: %s", profile.c_str());
    } else {
        nh.param("/NBV_selector/nbv_selector_profile", profile, profile);
        ROS_INFO("Received nbv_selector_profile as a yaml param: %s", profile.c_str());
        if (!profile.empty()) {
            ROS_WARN("nbv_selector_profile is not empty so will check for profile changes. This will override already set values");
            SettingProfileMethods(profile);
        }
    }
    CheckValues();
}

void NBVSelectorParameters::CheckValues(){
    // Check that weights are between 0 and 1
    if (image_component_weight < 0.0 || image_component_weight > 1.0) {
        throw std::runtime_error("Parameter 'image_component_weight' must be between 0.0 and 1.0");
    }
    if (metrics_component_weight < 0.0 || metrics_component_weight > 1.0) {
        throw std::runtime_error("Parameter 'metrics_component_weight' must be between 0.0 and 1.0");
    }
    if (quality_objective < 0.0 || quality_objective > 1.0) {
        throw std::runtime_error("Parameter 'quality_objective' must be between 0.0 and 1.0");
    }

    // Check that the methods' name exist
    if (_views_generation_methods.find(views_generation_method) == _views_generation_methods.end()) {
        throw std::runtime_error("Parameter 'views_generation_method' does not exist in the list of possible method. Please check your setup.");
    }
}

void NBVSelectorParameters::SettingProfileMethods(const std::string& profile){

    if(profile =="ours"){
        ROS_INFO("Profile is ours so will override views_generation_method and views_selection_method");
        views_generation_method="gradient";
        views_selection_method = "ours";
        image_component_weight = 1.0;
        metrics_component_weight = 1.0;
        use_distance_in_metrics_component = true; 
    }
    if(profile=="closest_frontiers"){
        views_generation_method="sphere";
        views_selection_method = "closest_frontiers";
        use_topology = false;
        distance_min = 3 ;
        distance_max = 8; 
        ROS_INFO("Profile is closest_frontiers so will override views_generation_method and views_selection_method");
    }

    if(profile=="count_frontiers"){
        views_generation_method="sphere";
        views_selection_method = "count_frontiers";
        use_topology = false;
        distance_min = 3 ;
        distance_max = 8; 
        ROS_INFO("Profile is highest_frontier so will override views_generation_method and views_selection_method");
    }

    if(profile=="speed"){
        views_generation_method="sphere";
        image_component_weight = 0.0;
        metrics_component_weight = 1.0;
        use_topology = false;
        use_distance_in_metrics_component = true; 
        distance_min = 3 ;
        distance_max = 8; 
        ROS_INFO("Profile is speed so will override views_generation_method, image_component_weight, metrics_component_weight and use_distance_in_metrics_component");
    }
}