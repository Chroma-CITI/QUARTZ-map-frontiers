#include <Eigen/Eigen>
#include <iostream>
#include "planning/modules/ViewGenerator.h"
#include "planning/modules/ViewEvaluator.h"
#include "planning/modules/MultiRobotCoordinator.h"
// #include <planning/modules/utils.h>
#include <planning/modules/NBVSelectorParameters.h>
#include "planning/data/type_conversions.h"
#include "planning/data/visualization_marker.h"
#include <string>
#include <math.h> 
#include <chrono>
#include <random>
#include <numeric>
#include <regex>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include "voxblox/utils/timing.h"
#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ptcloud_vis.h"
#include <voxblox_map/voxblox_map.h>
#include <voxblox_msgs/Layer.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include "planning_msgs/ReplanningStatus.h"
#include "planning_msgs/ReconfigurationPlan.h"



class NBV_Selector
{
private:
    /* data */
    voxblox_map::VoxbloxMap m_map;
    std::string world_frame_;
    std::string m_nbv_method; 

    ViewGenerator m_view_generator;
    std::vector<ViewCandidate> views;
    std::vector<ViewCandidate> views_history;
    std::unordered_set<Eigen::Vector3d, Vector3dHash, Vector3dEqual> frontiers_history_set;

    std::unordered_map<Eigen::Vector3d, int, Vector3dHash, Vector3dEqual> inacess_frontiers_cache; //0 is first time try, and 1 failed frontiers confirmed
    std::unordered_map<Eigen::Vector3d, int, Vector3dHash, Vector3dEqual> inacess_frontiers; //0 is first time try, and 1 failed frontiers confirmed


    std::vector<ViewCandidate> rejected_views;
    std::vector<Eigen::Vector3d> frontiers_set ;
    std::vector<Eigen::Vector3d> frontiers_subset ;

    std::unordered_map<Eigen::Vector3d, MetaCube, Vector3dHash, Vector3dEqual> meta_map ; 
    std::vector<Eigen::Vector3d> meta_frontiers ; 
    std::vector<Eigen::Vector3d> meta_holes ; 
    Eigen::Vector3d meta_neighbor_voxels_[26];
    // int metamap_x_min;
    // int metamap_x_max;
    // int metamap_y_min;
    // int metamap_y_max;
    // int metamap_z_min;
    // int metamap_z_max;

    ViewEvaluator m_view_evaluator;
    SensorModel m_sensor_model;
    ros::Time last_nbv_ ;

    bool remove_inacess_frontiers;
    bool use_topology;
    bool use_coordination;
    bool use_replanning;
    bool use_multi_resolution ; 
    bool m_use_repass ; 

    // System parameters
    NBVSelectorParameters _sys_params;

    //frontiers pointclouds
    pcl::PointCloud<pcl::PointXYZRGB> frontiers_pointcloud ;
    pcl::PointCloud<pcl::PointXYZRGB> frontiers_sub_pointcloud ;
    pcl::PointCloud<pcl::PointXYZRGB> frontiers_inacess_pointcloud ;
    pcl::PointCloud<pcl::PointXYZRGB> rejected_frontiers_pointcloud ;
    pcl::PointCloud<pcl::PointXYZRGB> values_for_eval_pointcloud ; 
    pcl::PointCloud<pcl::PointXYZRGB> values_for_eval_pointcloud2 ; 

    //meta frontiers pointcloud
    pcl::PointCloud<pcl::PointXYZRGB> meta_occupied_pointcloud ; 
    pcl::PointCloud<pcl::PointXYZRGB> meta_frontiers_pointcloud ; 
    pcl::PointCloud<pcl::PointXYZRGB> meta_holes_pointcloud ; 


    //views generated poses
    geometry_msgs::PoseArray views_set; 
    geometry_msgs::PoseArray rejected_views_set; 

    ViewCandidate m_current_goal;
    ViewCandidate m_current_pos;
    //Velocity angular or directional ?? 
    float m_vel_x ; 
    float m_vel_y ; 
    float m_vel_z ;

    bool m_availability;
    bool m_is_idle;
    bool m_frontiers_updated;
    int m_count;
    int counter_goal; 
    float m_update_map_tolerance ; 

    //ROS COMMUNICATION
    ros::NodeHandle n;
    ros::Subscriber sub_map_tsdf;
    ros::Subscriber sub_map_esdf;
    ros::Subscriber sub_pos;
    ros::Subscriber sub_idle;
    ros::Publisher pub_goal;
    ros::ServiceServer write_logs_server;
    ros::ServiceServer start_server;
    ros::ServiceServer stop_server;
    //frontiers and tsdfs
    ros::Publisher pub_pointcloud;
    ros::Publisher pub_frontiers;
    ros::Publisher pub_sub_frontiers;
    ros::Publisher pub_inacess_frontiers;
    ros::Publisher pub_rejected;
    ros::Publisher pub_test_values;
    ros::Publisher pub_test_values2;
    //views and selected views
    ros::Publisher pub_views ; 
    ros::Publisher pub_rejected_views ; 
    ros::Publisher pub_views_marker ; 
    ros::Publisher pub_views_rejected_marker ; 
    ros::Publisher pub_bounding_box_marker ; 
    ros::Publisher pub_nbv ; 
    //meta map
    ros::Publisher pub_meta_occupied ; 
    ros::Publisher pub_meta_frontiers ; 
    ros::Publisher pub_meta_holes ; 

    //multi-robot management
    std::map<int, ros::Subscriber> sub_other_nbv ; //should listen to the goal publisher of each other nbv_selector
    std::map<int, ros::Subscriber> sub_other_pos ; // should listen to the pos publisher of other robots
    ros::Publisher pub_replanning_status ; 
    std::map<int, ros::Subscriber> sub_replanning_status_multi ;
    ros::Publisher pub_new_configuration ; 
    ros::Publisher pub_new_plan_viz;
    ros::Subscriber sub_new_configuration ; 

    //TEAM AND COORDINATIONS ANALYSIS
    std::string m_drone_name ;
    int m_team_size;
    int m_team_id;
    std::vector<int> m_team;
    std::unordered_map<int, ViewCandidate> m_team_pos;
    std::unordered_map<int, ViewCandidate> m_team_goal;
    std::unordered_map<int, ViewCandidate> evaluate_allocation_input ; 

    bool use_repulsion_max ; 

    std::vector<int> m_replanning_status; 
    int m_forced_allocation_max_turns ; 

    MultiRobotCoordinator m_coordinator ; 

    bool frontiers_extended ; 

    //MULTI-RESOLUTION
    std::vector<QualityArea> m_quality_areas;





    //STATES
    const static unsigned char AVAILABLE = 0;  // NOLINT
    const static unsigned char BUSY = 1;      // NOLINT

    // GENERAL BEHAVIOUR
    bool is_started_ = false;

    //MEASURES
    float total_image ;
    float total_dist_angle ;
    int nb_views ;

public:
    NBV_Selector();
    NBV_Selector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, int team_id, std::vector<int> robot_team);
    void updateFrontiers();
    // void sample_subset_frontiers();
    void sample_subset_frontiers_discrete();
    void sample_subset_frontiers_shells();

    void update_frontiers_metamap();


    bool isFrontierVoxel_ESDF(const Eigen::Vector3d& voxel);

    bool isInBoundingBox(const Eigen::Vector3d& voxel);

    void extract_reached_frontiers(const ViewCandidate& next_best_view);

    void publishAllUpdatedTsdfVoxels() ;
    void publish_all_frontiers();
    void publish_sub_frontiers();
    void publish_inacess_frontiers();
    void publish_test_voxels();
    void publish_meta_map();
    void publish_replanning_status();
    void publish_new_configuration(); 


    void generate_views();
    void publish_views();
    void publish_goal();

    void select_NBV_method();
    void select_next_best_view(); 
    void next_best_view_closest_frontier();
    void next_best_view_count_frontier();
    void next_best_view_velocity();


    //Tests functions
    void visualize_voxels_ESDF();
    //void test_publish();

    void LogResults();

    //callbacks
    void tSDFCallback(const voxblox_msgs::Layer& layer_msg);
    void eSDFCallback(const voxblox_msgs::Layer& layer_msg);
    void posCallback(const nav_msgs::Odometry& msg_odom);
    void isIdleCallback(const std_msgs::Bool& msg_is_idle);
    //void multiRobotGoalCallback(const nav_msgs::Odometry& msg_odom, int robot_id);
    void multiRobotGoalCallback(const geometry_msgs::PoseStamped& msg, int robot_id);
    void multiRobotPosCallback(const nav_msgs::Odometry& msg_odom, int robot_id);



    void replanningStatusCallback(const planning_msgs::ReplanningStatus& msg_rs, int robot_id); 
    void replanningConfigurationCallback(const planning_msgs::ReconfigurationPlan& msg_rp);

    bool logCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT
    bool startCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT
    bool stopCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT
  

    ~NBV_Selector();
};

NBV_Selector::NBV_Selector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, int team_id, std::vector<int> robot_team)
{
    n = nh;
    //initialization

    m_drone_name = ros::this_node::getName();
    int num_drones = 0 ;
    nh.getParam("/number_of_drones", num_drones);

    //m_team_id = team_id ;
    //int m_team_id = m_drone_name.back() - '0'; //can only deal up to 9 robots
    std::regex re("iris_velodyne_(\\d+)");
    std::smatch match;

    if (std::regex_search(m_drone_name, match, re)) {
      m_team_id = std::stoi(match[1]);
    }

    ROS_INFO(" Robot name is %s", m_drone_name.c_str());
    ROS_INFO(" Robot id is %d ", m_team_id);
    //m_team_size = robot_team.size() ;
    m_team_size = num_drones; 
    for (int i = 0; i < m_team_size; i++)
    {
      m_team.push_back(i);
    }
    
    //m_team = robot_team;
    //m_team_pos;
    //m_team_goal;
    m_availability = AVAILABLE;
    m_frontiers_updated = true;
    m_is_idle = true;
    m_count =0;
    counter_goal = 0 ; 
    last_nbv_ = ros::Time::now();

    m_forced_allocation_max_turns = 2 ; //TO CHANGE 


    // Get ros params
    //taken for default value in the code of tsdf_map.h and esdf_map.h
    double voxel_size = 0.2;  // in m
    int voxels_per_side = 16;

    nh_private.param("voxel_size", voxel_size, voxel_size);
    ROS_INFO("Received voxel_size: %f found", voxel_size);

    nh_private.param("voxels_per_side", voxels_per_side, voxels_per_side);
    ROS_INFO("Received voxels_per_side: %i found", voxels_per_side);

    _sys_params.LoadFromRos(nh);

    world_frame_ = "world";
    //map
    m_map = voxblox_map::VoxbloxMap(voxel_size, voxels_per_side);

    //modules
    //m_view_generator.set_map(m_map);
    MetaCube::size = 1.0f ; 
    std::string method = _sys_params.views_generation_method;
    bool forced_safety = _sys_params.forced_safety;
    use_topology = _sys_params.use_topology ; 
    use_coordination = _sys_params.use_coordination ; 
    use_replanning = _sys_params.use_replanning ; 
    remove_inacess_frontiers = _sys_params.remove_inacess_frontiers;
    m_quality_areas = _sys_params.quality_areas_ ; 
    use_repulsion_max = _sys_params.use_repulsion_max ; 
    m_use_repass = _sys_params.use_repass ; 

    ROS_INFO("Received forced safety %s and remove inaccessible frontiers %s", forced_safety ? "true" : "false", remove_inacess_frontiers ? "true" : "false");

    if(use_coordination){
      m_coordinator = MultiRobotCoordinator(m_drone_name, m_team_size, m_team_id, _sys_params.dispersion_method, _sys_params.verbose_multi ) ;
    }

    m_view_generator = ViewGenerator(
      method,
      _sys_params.distance_min,
      _sys_params.distance_max,
      m_map,
      _sys_params.robot_radius,
      _sys_params.angle_low,
      _sys_params.angle_high,
      _sys_params.bounding_box,
      forced_safety,
      m_quality_areas,
      _sys_params.verbose_view_generation
    );
    m_sensor_model = SensorModel(
       _sys_params.p_ray_length,
       _sys_params.p_fov_x,
       _sys_params.p_fov_y,
       _sys_params.p_resolution_x,
       _sys_params.p_resolution_y,
       _sys_params.p_sampling_time
    );
    m_view_evaluator = ViewEvaluator(
      m_map,
      "",
      m_sensor_model,
      _sys_params.threshold_known,
      _sys_params.radius_surface_max,
      _sys_params.distance_min,
      _sys_params.distance_max,
      _sys_params.do_occlusions_check,
      _sys_params.angle_low,
      _sys_params.angle_high,
      _sys_params.bounding_box,
      m_quality_areas,
      _sys_params.verbose_view_evaluation
    ); 
    m_nbv_method = _sys_params.views_selection_method ; 

    if( m_nbv_method =="ours"){
      m_update_map_tolerance = 0; 
    }
    else{
      m_update_map_tolerance = MAXFLOAT; 
    }
    

    //frontiers
    frontiers_set = std::vector<Eigen::Vector3d>();
    frontiers_subset = std::vector<Eigen::Vector3d>();
    frontiers_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    frontiers_sub_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    frontiers_inacess_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    rejected_frontiers_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    values_for_eval_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    values_for_eval_pointcloud2 = pcl::PointCloud<pcl::PointXYZRGB>(); 

    //meta map
    meta_frontiers = std::vector<Eigen::Vector3d>();
    meta_holes = std::vector<Eigen::Vector3d>();
    // metamap_x_min = INT_MAX ; 
    // metamap_x_max = INT_MIN ;
    // metamap_y_min = INT_MAX ;
    // metamap_y_max = INT_MIN ;
    // metamap_z_min = INT_MAX ;
    // metamap_z_max = INT_MIN ;
    meta_neighbor_voxels_[0] = Eigen::Vector3d(1, 0, 0);
    meta_neighbor_voxels_[1] = Eigen::Vector3d(-1, 0, 0);
    meta_neighbor_voxels_[2] = Eigen::Vector3d(0, 1, 0);
    meta_neighbor_voxels_[3] = Eigen::Vector3d(0, -1, 0);
    meta_neighbor_voxels_[4] = Eigen::Vector3d(0, 0, 1);
    meta_neighbor_voxels_[5] = Eigen::Vector3d(0, 0, -1);
    
    meta_neighbor_voxels_[6] = Eigen::Vector3d(1, 0, -1);
    meta_neighbor_voxels_[7] = Eigen::Vector3d(1, 1, -1);
    meta_neighbor_voxels_[8] = Eigen::Vector3d(1, -1, -1);
    meta_neighbor_voxels_[9] = Eigen::Vector3d(1, 1, 0);
    meta_neighbor_voxels_[10] = Eigen::Vector3d(1, -1, 0);
    meta_neighbor_voxels_[11] = Eigen::Vector3d(1, 0, 1);
    meta_neighbor_voxels_[12] = Eigen::Vector3d(0, 1, 1);
    meta_neighbor_voxels_[13] = Eigen::Vector3d(0, -1, 1);
    meta_neighbor_voxels_[14] = Eigen::Vector3d(1, 1, 1);
    meta_neighbor_voxels_[15] = Eigen::Vector3d(0, 1, -1);
    meta_neighbor_voxels_[16] = Eigen::Vector3d(0, -1, -1);
    meta_neighbor_voxels_[17] = Eigen::Vector3d(1, -1, 1);
    meta_neighbor_voxels_[18] = Eigen::Vector3d(-1, 1, 0);
    meta_neighbor_voxels_[19] = Eigen::Vector3d(-1, -1, 0);
    meta_neighbor_voxels_[20] = Eigen::Vector3d(-1, 0, 1);
    meta_neighbor_voxels_[21] = Eigen::Vector3d(-1, 1, 1);
    meta_neighbor_voxels_[22] = Eigen::Vector3d(-1, -1, 1);
    meta_neighbor_voxels_[23] = Eigen::Vector3d(-1, 0, -1);
    meta_neighbor_voxels_[24] = Eigen::Vector3d(-1, 1, -1);
    meta_neighbor_voxels_[25] = Eigen::Vector3d(-1, -1, -1);

    //meta_frontiers
    meta_occupied_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>() ; 
    meta_frontiers_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 
    meta_holes_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>(); 

    //views
    views = std::vector<ViewCandidate>();
    views_history = std::vector<ViewCandidate>();
    //frontiers_history_set = std::unordered_set<Eigen::Vector3d>();
    frontiers_history_set = std::unordered_set<Eigen::Vector3d, Vector3dHash, Vector3dEqual>();
    views_set = geometry_msgs::PoseArray();
    rejected_views_set = geometry_msgs::PoseArray();




    //ros initialization
    // ros::init(argc, argv, "NBV_selector_node robot ");
    sub_map_tsdf = n.subscribe("tsdf_map_out", 1, &NBV_Selector::tSDFCallback, this);
    sub_map_esdf = n.subscribe("esdf_map_out", 1, &NBV_Selector::eSDFCallback, this);
    sub_pos = n.subscribe("groundtruth/odom", 2, &NBV_Selector::posCallback, this);
    sub_idle = n.subscribe("is_idle", 2, &NBV_Selector::isIdleCallback, this);
    pub_goal = n.advertise<geometry_msgs::PoseStamped>("pos_goal", 20); //to redefine msg type
    pub_pointcloud = n.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "test_point_cloud", 1, true);
    pub_frontiers = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "frontiers_point_cloud", 1, true);
    pub_sub_frontiers = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "subfrontiers_point_cloud", 1, true);

    pub_inacess_frontiers = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "inacess_point_cloud", 1, true);

    pub_rejected =  n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
            "rejected_point_cloud", 1, true);

    pub_test_values = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "occupied_point_cloud", 1, true);
    
    pub_test_values2 = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "unknown_point_cloud", 1, true);

    pub_views = n.advertise<geometry_msgs::PoseArray>("views", 1, true);
    pub_rejected_views = n.advertise<geometry_msgs::PoseArray>("rejected_views", 1, true);
    pub_views_marker = n.advertise<visualization_msgs::MarkerArray>("views_marker", 1, true);
    pub_views_rejected_marker = n.advertise<visualization_msgs::MarkerArray>("views_marker_rejected", 1, true);
    pub_bounding_box_marker = n.advertise<visualization_msgs::MarkerArray>("bounding_box_viz", 1, true);
    pub_nbv = n.advertise<geometry_msgs::Pose>("the_next_best_view", 1, true);

    pub_meta_occupied = n.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "meta_occupied_pointcloud", 1, true);
    pub_meta_frontiers = n.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "meta_frontiers_pointcloud", 1, true);
    pub_meta_holes = n.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "meta_holes_pointcloud", 1, true);

    pub_replanning_status = n.advertise<planning_msgs::ReplanningStatus>("replanning_status", 1);
    pub_new_configuration = n.advertise<planning_msgs::ReconfigurationPlan>("/reconfiguration_plan" , 1, true); 
    pub_new_plan_viz = n.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "new_plan_viz", 1, true);

    ROS_WARN_STREAM("goal: " << pub_goal.getTopic());
    ROS_WARN_STREAM("views: " << pub_views.getTopic());
    ROS_WARN_STREAM("replanning_status: " << pub_replanning_status.getTopic());
    ROS_WARN_STREAM("reconfiguration_plan: " << pub_new_configuration.getTopic());


    //team management
    //for(int i=0; i< m_team_size; i++){
    ROS_INFO(" IS MULTI ROBOT ? ");
    ROS_INFO(" %d ", m_team_size);
    ROS_INFO(" %d ", m_team.size());
    frontiers_extended = false;

    for(int i : m_team){

      m_replanning_status.push_back(0) ; 

      if(i != m_team_id){

        int temp_robot_id = i ; 

        //suscriber for the other robot goal
        ros::Subscriber robot_sub_goal ; 
        robot_sub_goal = 
        //n.subscribe( 

        // n.subscribe<nav_msgs::Odometry,
        //     const nav_msgs::Odometry::ConstPtr&>(
        //   "/iris_velodyne_" + std::to_string(i) +"/pos_goal" ,  
        //   1,
        //   [this, temp_robot_id](const nav_msgs::Odometry::ConstPtr& msg_ptr) {
        //         multiRobotGoalCallback(*msg_ptr, temp_robot_id);
        //   }

        n.subscribe<geometry_msgs::PoseStamped,
                const geometry_msgs::PoseStamped::ConstPtr&>(
                "/iris_velodyne_" + std::to_string(i) + "/next_best_viewpoint",
                1,
                [this, temp_robot_id](const geometry_msgs::PoseStamped::ConstPtr& msg_ptr) {
                    multiRobotGoalCallback(*msg_ptr, temp_robot_id);
                }

        
        );
        sub_other_nbv[i] = robot_sub_goal ;



        ros::Subscriber robot_sub_pos ; 
        robot_sub_pos = 
        //n.subscribe( 

        n.subscribe<nav_msgs::Odometry,
            const nav_msgs::Odometry::ConstPtr&>(
          "/iris_velodyne_" + std::to_string(i) +"/groundtruth/odom" ,  
          1,
          [this, temp_robot_id](const nav_msgs::Odometry::ConstPtr& msg_ptr) {
                multiRobotPosCallback(*msg_ptr, temp_robot_id);
          }

        
        );
        sub_other_pos[i] = robot_sub_pos ;
        
        
        //suscriber for the other robot replanning status
        ros::Subscriber robot_sub_rep_stat ; 
        robot_sub_rep_stat = 
        //n.subscribe( 
        n.subscribe<planning_msgs::ReplanningStatus,
            const planning_msgs::ReplanningStatus::ConstPtr&>(
          "/iris_velodyne_" + std::to_string(i) +"/replanning_status" ,  
          1,
          [this, temp_robot_id](const planning_msgs::ReplanningStatus::ConstPtr& msg_ptr) {
                replanningStatusCallback(*msg_ptr, temp_robot_id);
          }
        );
        sub_replanning_status_multi[i] = robot_sub_rep_stat ;




      }
    }

    sub_new_configuration = n.subscribe("/reconfiguration_plan", 2, &NBV_Selector::replanningConfigurationCallback, this);



    write_logs_server = n.advertiseService("log_NBV_selector", &NBV_Selector::logCallback, this);
    start_server = n.advertiseService("start_NBV_selector", &NBV_Selector::startCallback, this);
    stop_server = n.advertiseService("stop_NBV_selector", &NBV_Selector::stopCallback, this);

    total_image = 0 ;
    total_dist_angle = 0 ;
    nb_views = 0;

    // Visualize bounding box
    visualization_msgs::MarkerArray bounding_box_array;
    bounding_box_array.markers.push_back(map_frontiers::visualization::CreateBoundingBoxVisualization(_sys_params.bounding_box));
    int area_id = 0;
    for (const QualityArea& area : _sys_params.quality_areas_) {
      std::string area_namespace = "quality_area" + std::to_string(area_id);
      std::string quality_namespace = "quality_area_" + std::to_string(area_id) + "_value";
      bounding_box_array.markers.push_back(map_frontiers::visualization::CreateBoundingBoxVisualization(area.bounding_box, area_namespace, area_id));
      bounding_box_array.markers.push_back(map_frontiers::visualization::CreateBoundingBoxText(area.bounding_box, area.quality, quality_namespace, area_id));
      area_id++;
    }
    pub_bounding_box_marker.publish(bounding_box_array);

    //ros::spin()
}

// bool NBV_Selector::isFrontierVoxel_ESDF(const Eigen::Vector3d& voxel){
//   unsigned char voxel_state;
//   if ( m_frontier6 ) {
//     for (int i = 0; i < 6; ++i) {
//       voxel_state = m_map.getVoxelState_ESDF(voxel + c_neighbor_voxels_[i]);
//       if (voxel_state == voxblox_map::VoxbloxMap::UNKNOWN) {
//         continue;
//       }
//       if (m_surface_frontiers ) {
//         return voxel_state == voxblox_map::VoxbloxMap::OCCUPIED;
//       } else {
//         return true;
//       }
//     }
//   } else {
//     for (int i = 0; i < 26; ++i) {
//       voxel_state = m_map.getVoxelState_ESDF(voxel + c_neighbor_voxels_[i]);
//       if (voxel_state == voxblox_map::VoxbloxMap::UNKNOWN) {
//         continue;
//       }
//       if ( m_surface_frontiers ) {
//         return voxel_state == voxblox_map::VoxbloxMap::OCCUPIED;
//       } else {
//         return true;
//       }
//     }
//   }
//   return false;

// }


void NBV_Selector::updateFrontiers(){
    ros::Time start_update_frontiers = ros::Time::now();
    bool isfrontier;
    bool doublecheckfrontiers ;
    bool isLowConfidence ;
    unsigned char voxel_state ; 
    Eigen::Vector3d unknown_vox_frontier;
    int count_low_confidence_voxels = 0 ; 
    int low_confidence_frontiers = 0 ; 

    //meta_map
    Eigen::Vector3d current_meta_cube ; //coordinates of the meta_cube
    Eigen::Vector3d meta_cube_center ; 

    //set boolean about frontiers update to true
    m_frontiers_updated = true ; 

    unsigned char current_state;
    ROS_INFO_COND(_sys_params.verbose, "Updated frontiers: %lu found", frontiers_set.size());

    voxblox::BlockIndexList blocks;
    m_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);
    frontiers_pointcloud.clear();
    frontiers_set.clear();
    values_for_eval_pointcloud.clear();
    values_for_eval_pointcloud2.clear();
    frontiers_inacess_pointcloud.clear();

    //meta map
    meta_frontiers.clear();
    meta_occupied_pointcloud.clear();
    meta_frontiers_pointcloud.clear();

    Eigen::Vector3d current_pos = Eigen::Vector3d( m_current_pos.x, m_current_pos.y, m_current_pos.z);


    //multi-agent coordination
    if( m_nbv_method == "ours" && use_coordination ){

      for(int i=0; i<m_team_size; i++){
        if(i != m_team_id){

          Eigen::Vector3d other_robot_pos = Eigen::Vector3d(m_team_pos[i].x, m_team_pos[i].y, m_team_pos[i].z)  ; 
          if( (other_robot_pos - current_pos ).norm() < m_sensor_model.p_ray_length_ ){  //if there is another robot close, extend frontier search
            m_update_map_tolerance = MAXFLOAT; 
            frontiers_extended = true;

          } 

        }

      }

      if(use_replanning && m_replanning_status[m_team_id] == 2 ){ //if implementing a centroid based plan, extend frontier search
        m_update_map_tolerance = MAXFLOAT; 
        frontiers_extended = true ; 
      }





    }




    // Cache layer settings.
    size_t vps = m_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    for (const voxblox::BlockIndex& index : blocks) {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel>& block = m_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->getBlockByIndex(index);

      voxblox::Point origin = block.origin();

      for (size_t linear_index = 0; linear_index < num_voxels_per_block;
          ++linear_index) {
        voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
        const voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
        Eigen::Vector3d coord_3d = Eigen::Vector3d(coord.x(), coord.y(), coord.z());

        


        if( ((coord_3d - current_pos ).norm()) > m_sensor_model.p_ray_length_ + m_update_map_tolerance ){
          continue;
        }

        // ROS_INFO_COND(_sys_params.verbose, "TESTING COORDINATES %f %f %f", coord.x(), coord.y(), coord.z());

        //TOPOLOGY MAP INIT 
        if(use_topology){
          MetaCube::getMetaCube(coord_3d, current_meta_cube); //get the meta cube corresponding to the voxel
          meta_cube_center = MetaCube::getCenter(current_meta_cube);

          // ROS_INFO(" 3d point is %f %f %f", coord_3d.x(), coord_3d.y(), coord_3d.z());
          // ROS_INFO(" 3d metac is %f %f %f", current_meta_cube.x(), current_meta_cube.y(), current_meta_cube.z());
          //initialize meta map
          if (meta_map.find(meta_cube_center) != meta_map.end()) {
            // meta cub already initialized
          } else {
            if( meta_cube_center.z() >= 0 && isCorrectPos(meta_cube_center, _sys_params.bounding_box) && (m_map.getVoxelDistance_TSDF(meta_cube_center) >= 0) ){ //metamap is always above the ground
              meta_map[meta_cube_center] = MetaCube(0); 
            }
          }

        }
        //TOPOLOGY MAP INIT END
        
        isfrontier = m_view_evaluator.isSurfaceFrontier_TSDF(coord_3d, unknown_vox_frontier, voxel_state ) ;

        //IF REPASS
        if(m_use_repass){

          isLowConfidence = m_view_evaluator.isLowConfidenceVoxel(coord_3d) ; 
          if(isLowConfidence){

            frontiers_set.push_back( coord_3d );


            pcl::PointXYZRGB point;
            point.x = coord.x();
            point.y = coord.y();
            point.z = coord.z();
            point.r = 0;
            point.g = 0;
            point.b = 0;
            frontiers_pointcloud.push_back(point);
            
            count_low_confidence_voxels = count_low_confidence_voxels +1 ;
            if(isfrontier){
              low_confidence_frontiers = low_confidence_frontiers +1 ; 
            }

          }

        }

        if ( isfrontier ){

          //if frontiers already used for a selected view, removed from the frontiers set
          if ( frontiers_history_set.count(coord_3d) == 0 ){


            if( remove_inacess_frontiers == true ){


              //CHECK IF FAILED FRONTIERS
              auto it_ff = inacess_frontiers.find(coord_3d);
              if (it_ff != inacess_frontiers.end()) {

                  //frontier is a failed frontier
                  int ff_value = it_ff->second;
                  if(ff_value == 0 ){
                    inacess_frontiers[coord_3d] = 1 ; 
                  }


                  pcl::PointXYZRGB point;
                  point.x = coord_3d.x();
                  point.y = coord_3d.y();
                  point.z = coord_3d.z();
                  point.r = 255;
                  point.g = 0;
                  point.b = 0;
                  frontiers_inacess_pointcloud.push_back(point);

                  //continue;


              }
            }

            //END CHECK


            
            frontiers_set.push_back( coord_3d );

            // ROS_INFO_COND(_sys_params.verbose, "Frontier found distance %f", m_map.getVoxelDistance_TSDF( coord_3d ));

            pcl::PointXYZRGB point;
            point.x = coord.x();
            point.y = coord.y();
            point.z = coord.z();
            point.r = 0;
            point.g = 0;
            point.b = 0;
            frontiers_pointcloud.push_back(point);

            // pcl::PointXYZRGB point2;
            // point2.x = unknown_vox_frontier.x();
            // point2.y = unknown_vox_frontier.y();
            // point2.z = unknown_vox_frontier.z();
            // point2.r = 255;
            // point2.g = 0;
            // point2.b = 0;
            // values_for_eval_pointcloud2.push_back(point2);


          }
        }

        ///////////////////////////////////////////////////////////////////////////////////////////
        // //UPDATE OF THE META MAP 
        if(use_topology){

          
          //set meta cube at free if voxel at free and meta cube unknown
          if( voxel_state == voxblox_map::VoxbloxMap::FREE && meta_map[meta_cube_center].knowledge_status == 0 ){
            meta_map[meta_cube_center].knowledge_status = 1;
          }

          //set meta cube at occupied if meta cube is unknown or free and voxel occupied
          if( voxel_state == voxblox_map::VoxbloxMap::OCCUPIED && (meta_map[meta_cube_center].knowledge_status == 0 || meta_map[meta_cube_center].knowledge_status == 1) ){
            meta_map[meta_cube_center].knowledge_status = 2;

            pcl::PointXYZRGB point;
            point.x = meta_cube_center.x();
            point.y = meta_cube_center.y();
            point.z = meta_cube_center.z();
            point.r = 255;
            point.g = 0;
            point.b = 0;

            meta_occupied_pointcloud.push_back(point);

          }

          //in any case if voxel is frontier, set meta cube at frontier
          if( isfrontier){
            meta_map[meta_cube_center].knowledge_status = 3;

            meta_frontiers.push_back( meta_cube_center ) ; 
            std::array<Eigen::Vector3d, 8> corners ; 
            MetaCube::getCorners(current_meta_cube, corners);
            meta_frontiers.insert( meta_frontiers.end(), corners.begin(), corners.end() );


            pcl::PointXYZRGB point;
            point.x = meta_cube_center.x();
            point.y = meta_cube_center.y();
            point.z = meta_cube_center.z();
            point.r = 0;
            point.g = 255;
            point.b = 0;
            meta_frontiers_pointcloud.push_back(point);

            for( int i = 0 ; i < 8 ; i++){
              pcl::PointXYZRGB point;
              point.x = corners[i].x();
              point.y = corners[i].y();
              point.z = corners[i].z();
              point.r = 0;
              point.g = 255;
              point.b = 0;


              meta_frontiers_pointcloud.push_back(point);


            }


          }

        }
        //END UPDATE OF THE META MAP
        ///////////////////////////////////////////////////////////////////////////////////////////////////

        if( _sys_params.extra_viz){
          //empty pointcloud
          current_state = m_map.getVoxelState_TSDF(coord_3d, _sys_params.threshold_known);
          if ( current_state == voxblox_map::VoxbloxMap::FREE ){

            pcl::PointXYZRGB point;
            point.x = coord.x();
            point.y = coord.y();
            point.z = coord.z();
            point.r = 0;
            point.g = 0;
            point.b = 0;
            values_for_eval_pointcloud.push_back(point);
          }

          ///unknown pointcloud
          // if ( current_state == voxblox_map::VoxbloxMap::UNKNOWN ){

          //   pcl::PointXYZRGB point;
          //   point.x = coord.x();
          //   point.y = coord.y();
          //   point.z = coord.z();
          //   point.r = 0;
          //   point.g = 0;
          //   point.b = 0;
          //   values_for_eval_pointcloud2.push_back(point);
          // }
        }

      }

    //block.voxel_size()
    }
    ROS_INFO_ONCE("Frontiers updated!");
    ros::Time end_update_frontiers = ros::Time::now();
    ros::Duration duration = end_update_frontiers - start_update_frontiers;
    ROS_INFO_COND(_sys_params.timer, "[NBV_Selector][updateFrontiers] %.4f s", duration.toSec());



    //Detect additional metafrontiers
    if(use_topology){
      ros::Time start_update_metaholes = ros::Time::now();
      update_frontiers_metamap();
      ros::Time end_update_metaholes = ros::Time::now();
      ros::Duration duration = end_update_metaholes - start_update_metaholes;
      ROS_INFO_COND(_sys_params.timer, "[NBV_Selector][updateMetaHoles] %.4f s", duration.toSec());



    }

    


    //Clean failed frontiers
    for (auto it = inacess_frontiers.begin(); it != inacess_frontiers.end(); ) {
      if (it->second == 0) {
        it = inacess_frontiers.erase(it);  // erase returns the next valid iterator
      } else {
        ++it;
      }
    }

        
    ROS_WARN("Low confidence voxels are %d including %d low confidence frontiers", count_low_confidence_voxels, low_confidence_frontiers) ; 


    

 }

 void NBV_Selector::update_frontiers_metamap(){

  ROS_INFO("Metamap size before is %d", meta_map.size());


  meta_holes.clear();
  meta_holes_pointcloud.clear();

  for (const auto& pair : meta_map) {
    const Eigen::Vector3d& key = pair.first;
    const MetaCube& value = pair.second;

    bool closeSurface = false;
    bool isUnknown = ( value.knowledge_status == 0) ;
    if ( !isUnknown ){
      continue ; 
    }

    //to remove just for test
    // meta_holes.push_back( key) ; 

    // pcl::PointXYZRGB point;
    // point.x = key.x();
    // point.y = key.y();
    // point.z = key.z();
    // point.r = 0;
    // point.g = 0;
    // point.b = 255;
    // meta_holes_pointcloud.push_back(point);
    // continue;
    //end test


    //Check all the neighbours
    for(int i=0; i<26;i++){

          //check if in meta map
          Eigen::Vector3d meta_neighbor_3i = meta_neighbor_voxels_[i];
          meta_neighbor_3i = meta_neighbor_3i + key ;   
          // ROS_INFO(" Neighbor examined is %f %f %f for %f %f %f", meta_neighbor_3i.x() , meta_neighbor_3i.y(), meta_neighbor_3i.z(), key.x(), key.y(), key.z() );
          if (meta_map.find(meta_neighbor_3i) != meta_map.end()) {
            // ROS_INFO(" Found !" );
            MetaCube meta_neighbor = meta_map[meta_neighbor_3i] ; 
            if ( meta_neighbor.knowledge_status == 2 ){
              ROS_INFO(" Found meta hole !" );
              closeSurface = true;
              meta_neighbor.knowledge_status = 4; 
              meta_holes.push_back( key) ; 

              //publication

              pcl::PointXYZRGB point;
              point.x = key.x();
              point.y = key.y();
              point.z = key.z();
              point.r = 0;
              point.g = 0;
              point.b = 255;
              meta_holes_pointcloud.push_back(point);
              
              // std::array<Eigen::Vector3d, 8> corners ; 
              // MetaCube::getCorners(key, corners);

              // for( int i = 0 ; i < 8 ; i++){
              //   pcl::PointXYZRGB point;
              //   point.x = corners[i].x();
              //   point.y = corners[i].y();
              //   point.z = corners[i].z();
              //   point.r = 122;
              //   point.g = 122;
              //   point.b = 255;


              //   meta_holes_pointcloud.push_back(point);


              // }

              //END PUBLICATION 
              break; 

            }

          }

    }
    
  }

  ROS_INFO("Metamap size after is %d", meta_map.size());
  ROS_INFO("Metaholes size is %d", meta_holes.size());
  ROS_INFO("Metaholes pointcloud size is %d", meta_holes_pointcloud.size());




 }


//  void NBV_Selector::update_range_meta_map(int x, int y, int z){
//   if(metamap_x_max < x){
//     metamap_x_max = x ; 
//   }
//   if(metamap_y_max < y){
//     metamap_y_max = y ; 
//   }
//   if(metamap_z_max < z){
//     metamap_z_max = z ; 
//   }

//   if(metamap_x_min > x){
//     metamap_x_min = x ; 
//   }
//   if(metamap_y_min > y){
//     metamap_y_min = y ; 
//   }
//   if(metamap_z_min > z){
//     metamap_z_max = z ; 
//   }
  

//  }


 void NBV_Selector::visualize_voxels_ESDF(){
  ros::Time start_update_frontiers = ros::Time::now();

  unsigned char current_state;
  ROS_INFO_COND(_sys_params.verbose, "Updated frontiers: %lu found", frontiers_set.size());
  int i = 0;

  voxblox::BlockIndexList blocks;
  m_map.get_esdf_map_pointer()->getEsdfLayerPtr()->getAllAllocatedBlocks(&blocks);
  values_for_eval_pointcloud.clear();
  values_for_eval_pointcloud2.clear();

  // Cache layer settings.
  size_t vps = m_map.get_esdf_map_pointer()->getEsdfLayerPtr()->voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  for (const voxblox::BlockIndex& index : blocks) {
  // Iterate over all voxels in said blocks.
  const voxblox::Block<voxblox::EsdfVoxel>& block = m_map.get_esdf_map_pointer()->getEsdfLayerPtr()->getBlockByIndex(index);

    voxblox::Point origin = block.origin();

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
        ++linear_index) {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const voxblox::EsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
      Eigen::Vector3d coord_3d = Eigen::Vector3d(coord.x(), coord.y(), coord.z());

      //empty pointcloud
      current_state = m_map.getVoxelState_ESDF(coord_3d);
      if ( current_state == voxblox_map::VoxbloxMap::FREE ){

        pcl::PointXYZRGB point;
        point.x = coord.x();
        point.y = coord.y();
        point.z = coord.z();
        point.r = 0;
        point.g = 0;
        point.b = 0;
        values_for_eval_pointcloud.push_back(point);
        i=i+1;
      }

      //unknown pointcloud
      // if ( current_state == voxblox_map::VoxbloxMap::UNKNOWN ){

      //   pcl::PointXYZRGB point;
      //   point.x = coord.x();
      //   point.y = coord.y();
      //   point.z = coord.z();
      //   point.r = 0;
      //   point.g = 0;
      //   point.b = 0;
      //   values_for_eval_pointcloud2.push_back(point);
      // }

    }

  //block.voxel_size()
  }

  ROS_INFO_COND(_sys_params.verbose, "Nb free voxels is  %d", i);

}



// void NBV_Selector::sample_subset_frontiers(){
//   ros::Time start_sample_subset_frontiers = ros::Time::now();
//   frontiers_sub_pointcloud.clear();
//   frontiers_subset.clear();

//   std::vector<float> distances_table(frontiers_set.size());

//   if( frontiers_set.size() > _sys_params.subsampling_views ){

//     double min_value_distance = std::numeric_limits<double>::max() ; 
//     double max_value_distance = std::numeric_limits<double>::min() ; 
//     double total_distance = 0 ;

//     for(int i=0; i< frontiers_set.size(); i++){
      
//       Eigen::Vector3d current_pos = Eigen::Vector3d( m_current_pos.x, m_current_pos.y, m_current_pos.z );
//       double distance_frontier = (frontiers_set[i] - current_pos).norm();
//       distances_table[i] = distance_frontier ; 

//       if(distance_frontier > max_value_distance){
//         max_value_distance = distance_frontier;
//       }
//       if(distance_frontier < min_value_distance){
//         min_value_distance = distance_frontier;
//       }

//     }

//     float threshold_tirage = _sys_params.subsampling_views / frontiers_set.size() ;
//     float value_tirage = -1 ; 
//     int i = 0 ; 
//     std::vector<int> history_table(_sys_params.subsampling_views);
//     int history_count = 0;
//     while((frontiers_subset.size() < _sys_params.subsampling_views) && (i < frontiers_set.size() )){

//         value_tirage = (static_cast<float>(rand()) / RAND_MAX) ; 
//         threshold_tirage = _sys_params.subsampling_views / frontiers_set.size() ;
//         threshold_tirage = threshold_tirage *  ( (max_value_distance - distances_table[i] ) / (max_value_distance - min_value_distance )); 
        
//         ROS_INFO_COND(_sys_params.verbose, "[Sampling] in the while loop ... %d", i);
//         ROS_INFO_COND(_sys_params.verbose, "[Sampling] in the while loop ... %d", frontiers_subset.size());
//         if(value_tirage > threshold_tirage){

//           frontiers_subset.push_back(frontiers_set[i]);
//           history_table.push_back( i );
//           pcl::PointXYZRGB point;
//           point.x = frontiers_set[i].x();
//           point.y = frontiers_set[i].y();
//           point.z = frontiers_set[i].z();
//           point.r = 0;
//           point.g = 0;
//           point.b = 0;
//           frontiers_sub_pointcloud.push_back(point);
          

//         }
//         i=i+1;
//         ROS_INFO_COND(_sys_params.verbose, "[Sampling] End while loop");

//     }
    


//   }
//   else {

//     frontiers_subset = frontiers_set ; //careful ! Seems like a deep copy but not sure
//   }

//   ros::Time end_sample_subset_frontiers = ros::Time::now();
//   ros::Duration duration = end_sample_subset_frontiers - start_sample_subset_frontiers;
//   ROS_INFO_COND(_sys_params.timer, "[NBV_Selector][sample_subset_frontiers] %.4f s", duration.toSec());
// }

void NBV_Selector::sample_subset_frontiers_shells(){
  ros::Time start_sample_subset_frontiers = ros::Time::now();
  int sample_value = _sys_params.subsampling_views; 
  std::vector<int> history_table;
  ROS_INFO_COND(_sys_params.verbose,"[Sampling] sampling value is %d out of %d", sample_value, frontiers_set.size());
  if( _sys_params.subsampling_views > frontiers_set.size()){
    sample_value = frontiers_set.size();
  }
  // ROS_INFO_COND(_sys_params.verbose,"[Sampling] sampling value is %d", sample_value);

  ROS_INFO_COND(_sys_params.verbose, "[Sampling] shells start");
  float value_tirage = (static_cast<float>(rand()) / RAND_MAX) ; 
  int index_id = -1 ; 
  frontiers_sub_pointcloud.clear();
  frontiers_subset.clear();

  for(int i =0 ; i < sample_value; i++ ){

    value_tirage = (static_cast<float>(rand()) / RAND_MAX) * (frontiers_set.size() -1) ; 
    index_id = static_cast<int>(value_tirage) ;
    history_table.push_back(index_id);
    frontiers_subset.push_back( frontiers_set[index_id] ) ; 
    frontiers_sub_pointcloud.push_back( frontiers_pointcloud.points[index_id] );

  }

  ROS_INFO_COND(_sys_params.verbose, "[Sampling] shells end");
  if (_sys_params.verbose)
    printVectorOneLine(history_table);
  ros::Time end_sample_subset_frontiers = ros::Time::now();
  ros::Duration duration = end_sample_subset_frontiers - start_sample_subset_frontiers;
  ROS_INFO_COND(_sys_params.timer, "[NBV_Selector][sample_subset_frontiers_shells] %.4f s", duration.toSec());
}

void NBV_Selector::sample_subset_frontiers_discrete(){
  ros::Time start_sample_subset_frontiers = ros::Time::now();

  frontiers_sub_pointcloud.clear();
  frontiers_subset.clear();

  std::vector<float> distances_table(frontiers_set.size());
  std::vector<float> weight_table(frontiers_set.size());

  if( frontiers_set.size() > _sys_params.subsampling_views ){

    double min_value_distance = std::numeric_limits<double>::max() ; 
    double max_value_distance = std::numeric_limits<double>::min() ; 
    double total_distance = 0 ;

    for(int i=0; i< frontiers_set.size(); i++){
      
      Eigen::Vector3d current_pos = Eigen::Vector3d( m_current_pos.x, m_current_pos.y, m_current_pos.z );
      double distance_frontier = (frontiers_set[i] - current_pos).norm();
      distances_table[i] = distance_frontier ; 
      weight_table[i] = 1/( distance_frontier + 1e-6);

      if(distance_frontier > max_value_distance){
        max_value_distance = distance_frontier;
      }
      if(distance_frontier < min_value_distance){
        min_value_distance = distance_frontier;
      }

    }
    ROS_INFO_COND(_sys_params.verbose, "[Sampling][Before generating distribution] ");

    float sum_weight = std::accumulate( weight_table.begin(), weight_table.end(), 0); // sum of weights
    for(auto& w : weight_table){ w = w / sum_weight; } //normalize weights

    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> dist(weight_table.begin(), weight_table.end());

    ROS_INFO_COND(_sys_params.verbose, "[Sampling][After generating distribution] ");

    for(int i =0; i < _sys_params.subsampling_views ; i++){

          int idx = dist(gen);

          frontiers_subset.push_back(frontiers_set[idx]);

          pcl::PointXYZRGB point;
          point.x = frontiers_set[idx].x();
          point.y = frontiers_set[idx].y();
          point.z = frontiers_set[idx].z();
          point.r = 0;
          point.g = 0;
          point.b = 0;
          frontiers_sub_pointcloud.push_back(point);
          

    }
        
    
    


  }
  else {

    frontiers_subset = frontiers_set ; 
  }


  ros::Time end_sample_subset_frontiers = ros::Time::now();
  ros::Duration duration = end_sample_subset_frontiers - start_sample_subset_frontiers;
  ROS_INFO_COND(_sys_params.timer, "[NBV_Selector][sample_subset_frontiers_discrete] %.4f s", duration.toSec());


}

void NBV_Selector::LogResults() 
{
  // Log explored voxels during exploration
  float truncation_dist = 0.5; //CAREFUL TRUNCATION DIST IN METERS !!!
  int truncation_number = truncation_dist / m_map.getVoxelSize() ;
  float nb_voxels = m_map.countvoxelsinmap_TSDF(_sys_params.bounding_box, truncation_number); 
  ROS_WARN("Surface voxels in the map are %f", nb_voxels);

  float weight_saturation = 1;
  float maxx = -1 ;
  float minn = -1 ;
  int analyzed_voxels;
  float weight_average = m_map.evaluation_TSDF(truncation_dist, maxx, minn, analyzed_voxels, _sys_params.bounding_box, weight_saturation);
  ROS_INFO("Max weight is %f ", maxx);
  ROS_INFO("Min weight is %f ", minn);
  ROS_INFO("Nb voxels analyzed is %d ", analyzed_voxels);

  ROS_INFO("Voxel size is %f ", m_map.getVoxelSize());
  ROS_WARN("Weight average in map is %f", weight_average);

  std::ofstream out("/root/.ros/log/maps/explored_voxels.txt", std::ios::app);  // append mode
  if (out.is_open()) {
      out << "Surface voxels in the map: " << nb_voxels << "\n";
      out << "Max weight: " << maxx << "\n";
      out << "Min weight: " << minn << "\n";
      out << "Nb voxels analyzed: " << analyzed_voxels << "\n";
      out << "Weight average in map: " << weight_average << "\n\n";
      out.close();
  } else {
      ROS_WARN("[NBV_Selector] Failed to open file for destructor log.");
  }
}

NBV_Selector::~NBV_Selector()
{
}


void NBV_Selector::publishAllUpdatedTsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud_d;
  createDistancePointcloudFromTsdfLayer(
      *m_map.get_tsdf_map_pointer()->getTsdfLayerPtr(), &pointcloud_d);
  pointcloud_d.header.frame_id = world_frame_;
  pub_pointcloud.publish(pointcloud_d);

  // // Create a pointcloud with gradient direction = intensity.
  // pcl::PointCloud<pcl::PointXYZI> pointcloud_g;
  // createGradientPointcloudFromTsdfLayer(
  //     m_map.get_tsdf_map_pointer()->getTsdfLayerPtr(), &pointcloud_g);
  // pointcloud_g.header.frame_id = world_frame_;
  // gsdf_pointcloud_pub_.publish(pointcloud_g);
}

void NBV_Selector::publish_all_frontiers(){
  frontiers_pointcloud.header.frame_id = world_frame_;
  pub_frontiers.publish(frontiers_pointcloud);
}

void NBV_Selector::publish_test_voxels(){
  values_for_eval_pointcloud.header.frame_id = world_frame_;
  values_for_eval_pointcloud2.header.frame_id = world_frame_;
  pub_test_values.publish( values_for_eval_pointcloud);
  pub_test_values2.publish( values_for_eval_pointcloud2);
}

void NBV_Selector::publish_sub_frontiers(){
  frontiers_sub_pointcloud.header.frame_id = world_frame_;
  pub_sub_frontiers.publish(frontiers_sub_pointcloud);

  //Analyse ESDF
}

void NBV_Selector::publish_inacess_frontiers(){
  frontiers_inacess_pointcloud.header.frame_id = world_frame_;
  pub_inacess_frontiers.publish(frontiers_inacess_pointcloud);

  //Analyse ESDF
}

void NBV_Selector::publish_meta_map(){
  meta_occupied_pointcloud.header.frame_id = world_frame_;
  meta_frontiers_pointcloud.header.frame_id = world_frame_;
  meta_holes_pointcloud.header.frame_id = world_frame_;

  pub_meta_occupied.publish(meta_occupied_pointcloud);
  pub_meta_frontiers.publish(meta_frontiers_pointcloud);
  pub_meta_holes.publish(meta_holes_pointcloud);
  

}

void NBV_Selector::publish_replanning_status(){
  planning_msgs::ReplanningStatus msg ; 
  msg.robot_id = m_team_id ; 
  msg.replanning_status = m_replanning_status[m_team_id] ;
  ROS_WARN(" Robot %d publishing its planning status of %d", m_team_id, m_replanning_status[m_team_id]); 
  pub_replanning_status.publish(msg);

}

//CAREFUL : ID FOR ROBOTS HAVE TO BE INTS FOR IT TO WORK
void NBV_Selector::publish_new_configuration(){
  planning_msgs::ReconfigurationPlan msg; 
  pcl::PointCloud<pcl::PointXYZRGB> new_plan_viz = pcl::PointCloud<pcl::PointXYZRGB>(); 

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = world_frame_ ;
  msg.sender_id = m_team_id;  

  new_plan_viz.header.frame_id = world_frame_;

  for(int i=0; i< m_team_size; i++){
    geometry_msgs::Point p;
    p.x = m_coordinator.get_assigned_centroids().at(i).x() ;
    p.y = m_coordinator.get_assigned_centroids().at(i).y() ;
    p.z = m_coordinator.get_assigned_centroids().at(i).z() ; 
    msg.assigned_centroids.push_back(p);

    pcl::PointXYZRGB point;
    point.x = p.x ; 
    point.y = p.y;
    point.z = p.z;
    point.r = (int)  (255 * i / m_team_size ) ;
    point.g = (int)  (255 * i / m_team_size ) ; 
    point.b = (int)  (255 * i / m_team_size ) ;
    new_plan_viz.push_back(point);


  }




  pub_new_configuration.publish(msg);
  pub_new_plan_viz.publish(new_plan_viz) ; 
}

// void NBV_Selector::publish_analysis_map(){

//   values_for_eval_pointcloud.header.frame_id = world_frame_;
//   values_for_eval_pointcloud2.header.frame_id = world_frame_;
//   pub_test_values.publish( values_for_eval_pointcloud);
//   pub_test_values2.publish( values_for_eval_pointcloud2);
// }

void NBV_Selector::tSDFCallback(const voxblox_msgs::Layer& layer_msg){
  ROS_INFO_COND(_sys_params.verbose, "[TSDF callback] Entering TSDF callback");
  if (!is_started_)
    return;
  ROS_INFO_COND(_sys_params.verbose, "[TSDF callback] Activated "); 

  voxblox::timing::Timer receive_map_timer("map/receive_tsdf");

  bool success =
      voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(layer_msg, m_map.get_tsdf_map_pointer()->getTsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "MAP FRONTIERS : Got an invalid TSDF map message!");
    LOG(ERROR) << "layer_msg voxel size = " << layer_msg.voxel_size << " map layer voxel size = " << m_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->voxel_size();
    LOG(ERROR) << "layer_msg voxel per side = " << layer_msg.voxels_per_side << " map layer voxel per side = " << m_map.get_tsdf_map_pointer()->getTsdfLayerPtr()->voxels_per_side();
  } else {
    ROS_INFO_COND(_sys_params.verbose, "[TSDF callback] Got an TSDF map from ROS topic!");
    // publishAllUpdatedTsdfVoxels();
    // ROS_INFO_COND(_sys_params.verbose, "[TSDF callback] Published pointclouds");

    updateFrontiers();
    
    //if ours, extend the range of frontiers search if no frontier is found
    //for all no ours, there's no limited range of search
    if( m_nbv_method =="ours" ){
      
      if (frontiers_set.size() <= 20 && m_update_map_tolerance != MAXFLOAT  ){
        ROS_INFO("No frontiers found, extending map reach");
        m_update_map_tolerance = MAXFLOAT ; 
        updateFrontiers();
      }
      else{
        m_update_map_tolerance = 0 ;
      }

    }


    
    ROS_INFO_COND(_sys_params.verbose, "[TSDF callback] Updated frontiers ! ");

    if (_sys_params.extra_viz){
      publish_all_frontiers();
      ROS_INFO_COND(_sys_params.verbose, "Frontiers published !");
    }

    ROS_INFO_COND(_sys_params.verbose, "[TSDF callback] THERE ARE %d FRONTIERS", frontiers_set.size() );

    // publish_test_voxels(); 
    ROS_INFO_COND(_sys_params.verbose, "[TSDF callback] Published test voxels ! ");
    
    //sample frontiers
    sample_subset_frontiers_shells();


    ROS_INFO_COND(_sys_params.verbose, "[TSDF callback] SAMPLING");
    
    
    publish_sub_frontiers();
    if (_sys_params.extra_viz){
      publish_inacess_frontiers();
      if( use_topology){
        publish_meta_map();
      }
    }
    ROS_INFO_COND(_sys_params.verbose, "[TSDF callback] PUBLISHING SUB FRONTIERS");
  }
    //SEND PROCEDURE
    //voxblox_msgs::Layer layer_msg;
  


}

void NBV_Selector::eSDFCallback(const voxblox_msgs::Layer& layer_msg){
  ROS_INFO_COND(_sys_params.verbose, "[ESDF callback] Entering ESDF callback");
  if (!is_started_)
    return;
  ROS_INFO_COND(_sys_params.verbose, "[ESDF callback] Activated");
  voxblox::timing::Timer receive_map_timer("map/receive_esdf");

  bool success =
      voxblox::deserializeMsgToLayer<voxblox::EsdfVoxel>(layer_msg, m_map.get_esdf_map_pointer()->getEsdfLayerPtr());
  ROS_INFO_COND(_sys_params.verbose, "[ESDF callback] Deserialized done ! ");

  if (!success) {
    ROS_ERROR_THROTTLE(10, "MAP FRONTIERS : Got an invalid ESDF map message!");
    ROS_INFO_COND(_sys_params.verbose, "[ESDF callback] Deserialized failed ! ");
  } else {
    ROS_INFO_COND(_sys_params.verbose, "[ESDF callback] Deserialized sucess ! ");


    if (_sys_params.extra_viz){


      visualize_voxels_ESDF();
      //ROS_INFO_COND(_sys_params.verbose, "[ESDF callback] Sorted ESDF voxels ! ");
      publish_test_voxels(); 
      //ROS_INFO_COND(_sys_params.verbose, "[ESDF callback] Published test voxels ! ");
    }

    int number_views = 0 ;
    int number_resampling = 0 ; 

    // if( m_count ==0) {
    //   generate_views() ; 
    //   number_views = views.size();

    // }

    
    // if( m_count == 0 ){
    //   while( number_views == 0 && frontiers_set.size() > 0 ){
    //     number_resampling = number_resampling +1 ; 
    //     ROS_INFO_COND(_sys_params.verbose, "[ESDF callback] Found no views... Resampling for the %d times ", number_resampling);
    //     //sample frontiers
    //     sample_subset_frontiers_shells();
    //     generate_views() ; 
    //     number_views = views.size();


    //   }
    //   ROS_INFO_COND(_sys_params.verbose, "Views generated !" );
    //   ROS_INFO_COND(_sys_params.verbose,"Views published !");
    //   m_count = m_count +1 ; 

    // }
    // else{
    //   ROS_INFO_COND(_sys_params.verbose,"Publishing previous views !");
    // }

    
    // publish_views();

    }
  


}

void NBV_Selector::multiRobotGoalCallback( const geometry_msgs::PoseStamped& msg, int robot_id){
//void NBV_Selector::multiRobotGoalCallback(const nav_msgs::Odometry& msg_odom, int robot_id){ // TO FIX : Use tf/odometry in the multi robot case

  ViewCandidate vc ; 

  // ROS_INFO_COND(_sys_params.verbose, "CHECK 2 ");
  // vc.x = msg_odom.pose.pose.position.x ;
  // vc.y = msg_odom.pose.pose.position.y ;
  // vc.z = msg_odom.pose.pose.position.z ;
  // vc.q_x = msg_odom.pose.pose.orientation.x ;
  // vc.q_y = msg_odom.pose.pose.orientation.y ;
  // vc.q_z = msg_odom.pose.pose.orientation.z ;
  // vc.q_w = msg_odom.pose.pose.orientation.w ;


  vc.x   = msg.pose.position.x;
  vc.y   = msg.pose.position.y;
  vc.z   = msg.pose.position.z;

  vc.q_x = msg.pose.orientation.x;
  vc.q_y = msg.pose.orientation.y;
  vc.q_z = msg.pose.orientation.z;
  vc.q_w = msg.pose.orientation.w;

  m_team_goal[robot_id] = vc ; 
  evaluate_allocation_input[robot_id] = vc ; 
  ROS_INFO_STREAM(" Robot : " << m_drone_name) ;
  ROS_INFO("Received new goal of robot %d",robot_id) ;
  ROS_INFO("New goal is %f %f %f ", vc.x, vc.y, vc.z);

  if(robot_id == m_team_id){
    ROS_WARN("ERROR in multirobot topic suscription");


  }


}


void NBV_Selector::multiRobotPosCallback(const nav_msgs::Odometry& msg_odom, int robot_id){ 

  ViewCandidate vc ; 

  // ROS_INFO_COND(_sys_params.verbose, "CHECK 2 ");
  vc.x = msg_odom.pose.pose.position.x ;
  vc.y = msg_odom.pose.pose.position.y ;
  vc.z = msg_odom.pose.pose.position.z ;
  vc.q_x = msg_odom.pose.pose.orientation.x ;
  vc.q_y = msg_odom.pose.pose.orientation.y ;
  vc.q_z = msg_odom.pose.pose.orientation.z ;
  vc.q_w = msg_odom.pose.pose.orientation.w ;

  m_team_pos[robot_id] = vc ; 

  // ROS_INFO_STREAM(" Robot : " << m_drone_name) ;
  // ROS_INFO("Received new pos of robot %d",robot_id) ;
  // ROS_INFO("New pos is %f %f %f ", vc.x, vc.y, vc.z);

  if(robot_id == m_team_id){
    ROS_WARN("ERROR in multirobot topic suscription");


  }


}

void NBV_Selector::replanningStatusCallback(const planning_msgs::ReplanningStatus& msg_rs, int robot_id){

  if( msg_rs.robot_id != robot_id ){

    ROS_WARN("FATAL ERROR : Received a message on a topic for robot %d that is from robot %d", robot_id, msg_rs.robot_id);
  }

  m_replanning_status[msg_rs.robot_id] = msg_rs.replanning_status; 
  ROS_WARN(" I am robot %d", m_team_id);
  ROS_WARN("Robot %d planning status changed to %d", robot_id, msg_rs.replanning_status);


}

void NBV_Selector::replanningConfigurationCallback(const planning_msgs::ReconfigurationPlan& msg_rp){

  if(msg_rp.sender_id == m_team_id){
    ROS_WARN("Reconfiguration message is my own message : ignored ") ;
    return; 
  }

  if( m_replanning_status[m_team_id] != 0 ){
    ROS_WARN("Not ready to reconfigure : ignored ");
  }

  //reconfiguration
  std::vector<Eigen::Vector3d> external_reconfiguration_plan ; 
  for(int i=0; i<m_team_size; i++){

    Eigen::Vector3d assigned_centroid( msg_rp.assigned_centroids[i].x, msg_rp.assigned_centroids[i].y, msg_rp.assigned_centroids[i].z ); 
    external_reconfiguration_plan.push_back(assigned_centroid);

  }
  m_coordinator.set_assigned_centroids( external_reconfiguration_plan); 
  ROS_WARN("Robot %d : Reconfiguration message received ", m_team_id);

  for(int i=0; i< m_team_size; i++){
    ROS_WARN( "Received assigned centroids are  %d %f %f %f", i, m_coordinator.get_assigned_centroids().at(i)[0], m_coordinator.get_assigned_centroids().at(i)[1], m_coordinator.get_assigned_centroids().at(i)[2] );

  }
  


  m_replanning_status[m_team_id] = 2 ;


}

void NBV_Selector::posCallback(const nav_msgs::Odometry& msg_odom){ // TO FIX : Use tf/odometry in the multi robot case
  if (!is_started_)
      return;


  ///////////////// VERIFICATION OF MESSAGE INTEGRITY 

  if (std::isnan(msg_odom.twist.twist.linear.x)) {
      ROS_WARN("Warning: Odometry linear.x is NaN.");
  }
  if (std::isnan(msg_odom.twist.twist.linear.y)) {
      ROS_WARN("Warning: Odometry linear.y is NaN.");
  }
  if (std::isnan(msg_odom.twist.twist.linear.z)) {
      ROS_WARN("Warning: Odometry linear.z is NaN.");
  }

  // Check for NaN in twist.angular
  if (std::isnan(msg_odom.twist.twist.angular.x)) {
      ROS_WARN("Warning: Odometry angular.x is NaN.");
  }
  if (std::isnan(msg_odom.twist.twist.angular.y)) {
      ROS_WARN("Warning: Odometry angular.y is NaN.");
  }
  if (std::isnan(msg_odom.twist.twist.angular.z)) {
      ROS_WARN("Warning: Odometry angular.z is NaN.");
  }

  // Check for NaN in pose.position
  if (std::isnan(msg_odom.pose.pose.position.x)) {
      ROS_WARN("Warning: Odometry position.x is NaN.");
  }
  if (std::isnan(msg_odom.pose.pose.position.y)) {
      ROS_WARN("Warning: Odometry position.y is NaN.");
  }
  if (std::isnan(msg_odom.pose.pose.position.z)) {
      ROS_WARN("Warning: Odometry position.z is NaN.");
  }

  // Check for NaN in pose.orientation
  if (std::isnan(msg_odom.pose.pose.orientation.x)) {
      ROS_WARN("Warning: Odometry orientation.x is NaN.");
  }
  if (std::isnan(msg_odom.pose.pose.orientation.y)) {
      ROS_WARN("Warning: Odometry orientation.y is NaN.");
  }
  if (std::isnan(msg_odom.pose.pose.orientation.z)) {
      ROS_WARN("Warning: Odometry orientation.z is NaN.");
  }
  if (std::isnan(msg_odom.pose.pose.orientation.w)) {
      ROS_WARN("Warning: Odometry orientation.w is NaN.");
  }

  // Check if the message timestamp is zero
  if (msg_odom.header.stamp.isZero()) {
      ROS_WARN("Warning: Odometry message has no timestamp.");
  }

/////////////////////////////////////////////////////////////////


  ROS_INFO_COND(_sys_params.verbose, "POS CALLBACK");
  m_current_pos.o_x = m_current_pos.x ; 
  m_current_pos.o_y = m_current_pos.y;
  m_current_pos.o_z = m_current_pos.z;
  // ROS_INFO_COND(_sys_params.verbose, "CHECK 2 ");
  m_current_pos.x = msg_odom.pose.pose.position.x ;
  m_current_pos.y = msg_odom.pose.pose.position.y ;
  m_current_pos.z = msg_odom.pose.pose.position.z ;
  m_current_pos.q_x = msg_odom.pose.pose.orientation.x ;
  m_current_pos.q_y = msg_odom.pose.pose.orientation.y ;
  m_current_pos.q_z = msg_odom.pose.pose.orientation.z ;
  m_current_pos.q_w = msg_odom.pose.pose.orientation.w ;
  
  // ROS_INFO_COND(_sys_params.verbose, "CHECK 3 ");
  //linear speed : should be angular ? 
  m_vel_x = msg_odom.twist.twist.linear.x ;
  m_vel_y = msg_odom.twist.twist.linear.y ;
  m_vel_z = msg_odom.twist.twist.linear.z ;
  // ROS_INFO_COND(_sys_params.verbose, "[First] VEL VALUES ARE %f %f %f", m_vel_x, m_vel_y, m_vel_z);
  // ROS_INFO_COND(_sys_params.verbose, "[First] POS VALUES ARE %f %f %f", m_current_pos.x,  m_current_pos.y ,  m_current_pos.z);
  // ROS_INFO_COND(_sys_params.verbose, "[First] OR VALUES ARE %f %f %f %f", m_current_pos.q_x,  m_current_pos.q_y ,  m_current_pos.q_z, m_current_pos.q_w );
  

    if( m_availability == BUSY){
      //Position
      float distance_difference = pow((m_current_pos.x - m_current_goal.x ), 2)   + pow((m_current_pos.y - m_current_goal.y ), 2) + pow((m_current_pos.z - m_current_goal.z ), 2) ; 
      bool isPositionCorrect = distance_difference < _sys_params.tolerance_distance;

      // Orientation
      Eigen::Quaterniond current_orientation(m_current_pos.q_x, m_current_pos.q_y, m_current_pos.q_z, m_current_pos.q_w ); 
      Eigen::Quaterniond goal_orientation(m_current_goal.q_x, m_current_goal.q_y, m_current_goal.q_z, m_current_goal.q_w ); 
      bool isOrientationCorrect = (current_orientation.isApprox(goal_orientation, _sys_params.angular_tolerance) || current_orientation.coeffs().isApprox( -goal_orientation.coeffs(), _sys_params.angular_tolerance));

      if( isPositionCorrect && isOrientationCorrect){
        m_availability = AVAILABLE ; 
        ROS_INFO_COND(_sys_params.verbose, "ROBOT IS NOW AVAILABLE");

        m_frontiers_updated = false;

        //PROCESSING FOR INACESSIBLE FRONTIERS
        if( remove_inacess_frontiers == true ){
          //inacess_frontiers.merge(inacess_frontiers_cache); // only c++17
          for (const auto& [key, value] : inacess_frontiers_cache) {
            inacess_frontiers[key] = value;  // insert or overwrite
          }
          inacess_frontiers_cache.clear();

        }


      }
    }


  ros::Duration duration = ros::Time::now() - last_nbv_ ; 
  // if the robot has been on a standstill for more than one sec, we allow it to have a new goal
  if( m_is_idle &&  ( duration.toSec() > 1.0 ) && (m_frontiers_updated == true )){
    m_availability = AVAILABLE ; 
  }

  //if the robot hasn't had a new goal in the last second and is available, find nbv
  if( (m_availability == AVAILABLE) && ( duration.toSec() > 1.0 ) && (m_frontiers_updated == true )) {
    ROS_INFO_COND(_sys_params.verbose, " Going into selection");

    generate_views() ; 
    ROS_INFO_COND(_sys_params.verbose, "Views generated in selection !" );
    publish_views();
    ROS_INFO_COND(_sys_params.verbose,"Views published in selection !");


    //select_next_best_view();
    select_NBV_method();
    // m_count = 0; //reset views selection
    last_nbv_ = ros::Time::now();


    if( m_team_goal.size() >= m_team_id ){ //robot doesn't publish goal and launch until all the previous have started
      publish_goal();
      m_availability = BUSY ; 
      m_frontiers_updated = false ;
      ROS_INFO_COND(_sys_params.verbose, "ROBOT IS NOW BUSY");
    }

    
  }
  else {
    ROS_INFO_COND(_sys_params.verbose, "ROBOT IS NOT AVAILABLE %f ", duration.toSec()  );
  }


}

void NBV_Selector::isIdleCallback(const std_msgs::Bool& msg_is_idle) {
  if (_sys_params.verbose && m_is_idle != msg_is_idle.data) {
    if (m_is_idle)
      ROS_INFO_COND(_sys_params.verbose,"drone went from %s to %s", "idle", "moving");
    else
      ROS_INFO_COND(_sys_params.verbose,"drone went from %s to %s", "moving", "idle");
  }
  m_is_idle = msg_is_idle.data;
}

bool NBV_Selector::logCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {  // NOLINT
  ROS_INFO("Received a log service call.");
  try
  {
    LogResults();
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return false;
  }
  return true;  // Return true to indicate the callback handled the response
}

bool NBV_Selector::startCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {  // NOLINT
  ROS_INFO("Received a start service call.");
  is_started_ = true;
  return true;  // Return true to indicate the callback handled the response
}

bool NBV_Selector::stopCallback(
    std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response&
    /*response*/) {  // NOLINT
  ROS_INFO("Received a stop service call.");
  is_started_ = false;
  return true;  // Return true to indicate the callback handled the response
}

void NBV_Selector::publish_goal(){ 

  geometry_msgs::PoseStamped next_goal ;

  next_goal.header.stamp = ros::Time::now();  // Set timestamp
  next_goal.header.frame_id = world_frame_;

  map_frontiers::conversions::ViewCandidateToGeometryPose(m_current_goal, next_goal.pose);
  if( views.size() == 0){
    ROS_WARN("Not publishing goal because no views found");
  }
  else{
    ROS_WARN("PUBLISHING GOAL NOW "); 
    pub_goal.publish(next_goal);
  }


}

void NBV_Selector::generate_views(){
  ros::Time start_generate_views = ros::Time::now();
  //m_view_generator.generateViews(frontiers_set);
  m_view_generator.generateViews(frontiers_subset);
  views = m_view_generator.getViewCandidates();
  ros::Time end_generate_views = ros::Time::now();
  ros::Duration duration =  end_generate_views - start_generate_views ;
  ROS_INFO_COND(_sys_params.timer, "[NBV_Selector][generate_views] %.4f s", duration.toSec());
}

void NBV_Selector::publish_views(){
  if (!_sys_params.extra_viz)
    return;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::MarkerArray marker_array_rejects;
  float range_for_viz = 0.3;

  views_set.poses.clear();
  rejected_views_set.poses.clear();

  
  //rejected_views
  rejected_views = m_view_generator.getRejectedViews() ; 
  for(int i = 0; i < rejected_views.size(); i++){
    geometry_msgs::Pose temp_view;
    map_frontiers::conversions::ViewCandidateToGeometryPose(rejected_views[i], temp_view);
    geometry_msgs::Point temp_frontier;
    temp_frontier.x = rejected_views[i].o_x;
    temp_frontier.y = rejected_views[i].o_y;
    temp_frontier.z = rejected_views[i].o_z;
    rejected_views_set.poses.push_back(temp_view);

    // Create markers to represent the fov the view would have
    marker_array_rejects.markers.push_back(map_frontiers::visualization::CreateHorizontalFOVMarker(temp_view, i, range_for_viz));
    marker_array_rejects.markers.push_back(map_frontiers::visualization::CreateVerticalFOVMarker(temp_view, i, range_for_viz));
    marker_array_rejects.markers.push_back(map_frontiers::visualization::CreateViewToFrontiersLine(temp_view, temp_frontier, i));


  }

  //History of selected views
  for(int i = 0; i < views_history.size(); i++){
    geometry_msgs::Pose temp_view;
    map_frontiers::conversions::ViewCandidateToGeometryPose(views_history[i], temp_view);
    geometry_msgs::Point temp_frontier;
    temp_frontier.x = views_history[i].o_x;
    temp_frontier.y = views_history[i].o_y;
    temp_frontier.z = views_history[i].o_z;
    views_set.poses.push_back(temp_view);

    // Create markers to represent the fov the view would have
    marker_array.markers.push_back(map_frontiers::visualization::CreateHorizontalFOVMarker(temp_view, i, range_for_viz));
    marker_array.markers.push_back(map_frontiers::visualization::CreateVerticalFOVMarker(temp_view, i, range_for_viz));
    marker_array.markers.push_back(map_frontiers::visualization::CreateViewToFrontiersLine(temp_view, temp_frontier, i));

    // ROS_INFO_COND(_sys_params.verbose, "View history %d %f %f %f %f", i, views_history[i].q_x, views_history[i].q_y, views_history[i].q_z, views_history[i].q_w);

  }
  
  for(int i = 0; i < views.size(); i++){
    geometry_msgs::Pose temp_view;
    map_frontiers::conversions::ViewCandidateToGeometryPose(views[i], temp_view);
    geometry_msgs::Point temp_frontier;
    temp_frontier.x = views[i].o_x;
    temp_frontier.y = views[i].o_y;
    temp_frontier.z = views[i].o_z;
    views_set.poses.push_back(temp_view);

    // Create markers to represent the fov the view would have
    marker_array.markers.push_back(map_frontiers::visualization::CreateHorizontalFOVMarker(temp_view, i, range_for_viz));
    marker_array.markers.push_back(map_frontiers::visualization::CreateVerticalFOVMarker(temp_view, i, range_for_viz));
    marker_array.markers.push_back(map_frontiers::visualization::CreateViewToFrontiersLine(temp_view, temp_frontier, i));

    // ROS_INFO_COND(_sys_params.verbose, "View  %d %f %f %f %f", i, views[i].q_x, views[i].q_y, views[i].q_z, views[i].q_w);

  }
  views_set.header.frame_id = world_frame_;
  rejected_views_set.header.frame_id = world_frame_;
  pub_views.publish(views_set);
  pub_rejected_views.publish( rejected_views_set);
  pub_views_marker.publish(marker_array);

  pub_views_rejected_marker.publish( marker_array_rejects ) ;



  //publishing of rejected frontiers
  std::vector<Eigen::Vector3d> rejected_frontiers = m_view_generator.getRejectedFrontiers();
  for(int i=0; i< rejected_frontiers.size(); i++){
    pcl::PointXYZRGB point;
    point.x = rejected_frontiers[i].x();
    point.y = rejected_frontiers[i].y();
    point.z = rejected_frontiers[i].z();
    point.r = 0;
    point.g = 0;
    point.b = 0;
    rejected_frontiers_pointcloud.push_back(point);

  }


  rejected_frontiers_pointcloud.header.frame_id = world_frame_;
  pub_rejected.publish(rejected_frontiers_pointcloud);




}

void NBV_Selector::select_NBV_method(){

  ROS_INFO_COND(_sys_params.verbose,"SELECTION METHOD IS %s", "m_nbv_method");

  if( m_nbv_method == "ours" ){
    select_next_best_view();
    return;
  }

  if( m_nbv_method == "closest_frontiers"){
    next_best_view_closest_frontier();
    return;
  }

  if( m_nbv_method == "count_frontiers"){
    next_best_view_count_frontier();
    return;
  }

  if( m_nbv_method == "speed"){
    select_next_best_view();
    ROS_WARN("Profile is speed ");
    return; 
  }
  
  ROS_WARN("Profile doesn't exist !! ");
  
  



}

//Called in poscallback after pose updated
void NBV_Selector::select_next_best_view(){

  //delayed start if multiple robots 
  if( use_coordination && ( m_team_goal.size() < m_team_id) ){
    ROS_WARN(" Drone %d can not make a decision : not enough information available ! ", m_team_id); 
    return;
  }
  
  ROS_INFO_COND(_sys_params.verbose_planning, "SELECTING VIEWS NOW");
  
  // ROS_INFO_COND(_sys_params.verbose, "EMPTY SPACE");

  //evaluate views
  std::vector<float> values_views;
  int index_of_nbv = -1 ;
  float max_value_nbv = -1 ; 
  float image_value = -1 ; 
  float total_value = -1 ;
  float value_angular = 0 ;
  float value_topology = 0 ; 
  float coordination_value = 0 ; 
  float dist_view = 0 ;
  float dist_min_views = -1 ;
  float dist_max_views = -1;
  std::vector<float> image_values ; 
  int max_nb_frontiers_visible = -1;
  int temp_nb_frontiers_visible = -1;
  int index_max_frontiers = -1;

  float repulsion_force = -10 ;

  //MULTI ROBOT
  if( m_team_size > 1 ){
    //COORDINATION PRE PROCESS
    ROS_WARN(" Robot %d of team size %d", m_team_id, m_team_size); 
    ROS_WARN(" Evaluate allocation size %d", evaluate_allocation_input.size()); 

    //IF NOT IN CORRECTION MODE 
    if( use_coordination && use_replanning && evaluate_allocation_input.size() >= m_team_size -1 ){

      ROS_WARN(" Views size %d", evaluate_allocation_input.size()); 

      for (const auto& kv : evaluate_allocation_input) {
        int i = kv.first;
        ROS_WARN( "Evaluate allocation  %d %f %f %f", i, evaluate_allocation_input.at(i).x, evaluate_allocation_input.at(i).y, evaluate_allocation_input.at(i).z );
      }




      float view_repartition = -1 ; 
      evaluate_allocation_input[m_team_id] = m_current_pos ; 
      view_repartition = m_coordinator.evaluate_allocation( evaluate_allocation_input , views) ; 
      const std::unordered_map<int, int> assigned_views = m_coordinator.get_assigned_views() ;

      ROS_WARN( "Current repartition value is %f ", view_repartition) ; 
      ROS_WARN( "Allocation are %d %d %d", assigned_views.at(0), assigned_views.at(1), assigned_views.at(2) );

      
      //if ( (view_repartition > ) && (m_coordinator.assigned_views[m_team_id] < ) ) {
      if( (assigned_views.at(m_team_id) < (1.0 / (m_team_size+1 ) * views.size() ) ) && (m_coordinator.isReconfigurationPossible(m_replanning_status) ) ) {

        //ask other robots not to replan
        ROS_WARN(" Entering replanning for robot %d", m_team_id); 
        m_replanning_status[m_team_id] = 1 ; 
        publish_replanning_status();


        ROS_WARN(" Re-clustering %f", m_team_id); 
        m_coordinator.balance_clustering(evaluate_allocation_input, views) ; 
        ROS_WARN(" Found %d centroids for a team of %d", m_coordinator.get_centroids().size(), m_team_size ); 

        for(int i=0; i< m_team_size; i++){
          ROS_WARN( "Centroids are  %d %f %f %f", i, m_coordinator.get_centroids().at(i)[0], m_coordinator.get_centroids().at(i)[1], m_coordinator.get_centroids().at(i)[2] );

        }
        ROS_WARN( "Centroids numbers  %d ", m_coordinator.get_centroids().size() );

        ROS_WARN( "Team size is %d  ", m_team_size );
        m_coordinator.update_centroid_distance_matrix( evaluate_allocation_input ) ; 

        ROS_WARN( "Distance matrix updated  " );


        //assignment hungarian
        m_coordinator.hungarian_algorithm_assignment();
        ROS_WARN( " Centroids assigned (hungarian) with cost %f ", m_coordinator.get_assignment_cost() );

        for(int i=0; i< m_team_size; i++){
          ROS_WARN( "Assigned centroids are  %d %f %f %f", i, m_coordinator.get_assigned_centroids().at(i)[0], m_coordinator.get_assigned_centroids().at(i)[1], m_coordinator.get_assigned_centroids().at(i)[2] );

        }

        //assignment nearest neighbor
        m_coordinator.nearest_neighbor_assignment();
        ROS_WARN( " Centroids assigned (nearest neighgor) with cost %f ", m_coordinator.get_assignment_cost() );

        for(int i=0; i< m_team_size; i++){
          ROS_WARN( "Assigned centroids are  %d %f %f %f", i, m_coordinator.get_assigned_centroids().at(i)[0], m_coordinator.get_assigned_centroids().at(i)[1], m_coordinator.get_assigned_centroids().at(i)[2] );

        }



        //send message to other robots
        m_replanning_status[m_team_id] = 2 ; 
        publish_replanning_status();
        publish_new_configuration(); 

      }
      

      
      
    }
  }
  ///////////////////////// END MULTI ROBOT



  Eigen::Vector3d current_pos_vector( m_current_pos.x ,m_current_pos.y , m_current_pos.z );
  Eigen::Vector3d vel( m_vel_x, m_vel_y, m_vel_z);
  // ROS_INFO_COND(_sys_params.verbose, "VEL VALUES ARE %f %f %f", m_vel_x, m_vel_y, m_vel_z);

  if (views.size() > 0){
    dist_min_views = m_view_evaluator.getMinMaxViewDistance(views, current_pos_vector, dist_max_views) ;
  }
  else{
    ROS_INFO_COND(_sys_params.verbose, "NO VIEWS TO CHECK FOR DISTANCE VIEWS ");
  }
  ROS_INFO_COND(_sys_params.verbose, "MIN DISTANCE FRONTIERS IS %f", dist_min_views);
  ROS_INFO_COND(_sys_params.verbose, "MAX DISTANCE FRONTIERS IS %f", dist_max_views);



  m_current_goal = m_current_pos;

  ROS_INFO_COND(_sys_params.verbose, "EXAMINING %d", views.size());
  ROS_INFO_COND(_sys_params.verbose, "image_component_weight %f metrics_component_weight %f use_distance_in_metrics_component %d", _sys_params.image_component_weight, _sys_params.metrics_component_weight, _sys_params.use_distance_in_metrics_component);
  ros::Time total_view_evaluation_start = ros::Time::now();

  ViewCandidate view ;
  bool integrity_view = false;
  for(int i=0;i< views.size();i++){

    ROS_INFO_COND(_sys_params.verbose, "GET VOXELS VIEW %d", i);


    view = views[i] ; 
    integrity_view = checkIntegrityView(view);

    // std::vector<Eigen::Vector3d> visible_voxels; 

    ros::Time start_get_visible_voxels_lidar = ros::Time::now();

    // m_view_evaluator.getVisibleVoxels_LIDAR(
    // &visible_voxels, pos, orient) ;
    // ros::Time end_get_visible_voxels_lidar = ros::Time::now();
    // ros::Duration duration = end_get_visible_voxels_lidar - start_get_visible_voxels_lidar;
    // ROS_INFO_COND(_sys_params.timer, "[NBV_Selector][getVisibleVoxels_LIDAR] %.4f s", duration.toSec());

    ROS_INFO_COND(_sys_params.verbose, "COUNTING FRONTIERS VIEW %d", i);
    
    ros::Time start_view_evaluation = ros::Time::now();
    image_value = m_view_evaluator.inverseRayCast(view, frontiers_set, temp_nb_frontiers_visible, inacess_frontiers, false ); 

    ros::Time end_view_evaluation = ros::Time::now();
    ros::Duration duration = end_view_evaluation - start_view_evaluation;
    // ROS_INFO_COND(_sys_params.timer, "[NBV_Selector][Evaluate View] %.4f s", duration.toSec());


    if ( temp_nb_frontiers_visible > max_nb_frontiers_visible){
      max_nb_frontiers_visible = temp_nb_frontiers_visible ; 
      index_max_frontiers = i;
    }
    image_values.push_back(image_value);
  }




  
  for(int i=0;i< views.size();i++){

    ROS_INFO_COND(_sys_params.verbose, "Max frontiers visible is %d",max_nb_frontiers_visible);

    view = views[i] ; 
    integrity_view = checkIntegrityView(view);

    Eigen::Vector3d view_vector = Eigen::Vector3d( view.x, view.y, view.z);
    Eigen::Quaterniond orient = Eigen::Quaterniond( view.q_x, view.q_y, view.q_z, view.q_w);

    image_values[i] = image_values[i] / max_nb_frontiers_visible ; 
    ROS_INFO_COND(_sys_params.verbose_planning, "IMAGE VALUE of  %d is %f", i, image_values[i]);
    total_image = total_image + image_values[i];

    value_angular = m_view_evaluator.evaluate_view_angular( view, current_pos_vector, vel ) ; 
    ROS_INFO_COND(_sys_params.verbose_planning, "ANGLE COST of  %d is %f", i, value_angular);

    //IF DISTANCE IS TAKEN  INTO ACCOUNT
    if ( _sys_params.use_distance_in_metrics_component == true ){
      dist_view = (view_vector - current_pos_vector ).norm() ;
      value_angular = m_view_evaluator.evaluate_distance_angle_cost( value_angular, dist_min_views, dist_view ) ; 
    }

    ROS_INFO_COND(_sys_params.verbose_planning, "ANGLE/DISTANCE VALUE of  %d is %f", i, value_angular);
    // temp_value_angular = ; 
    // ROS_INFO_COND(_sys_params.verbose, "DIST/ANGLE COST VALUE of  %d is %f", i, temp_value_angular);
    total_dist_angle = total_dist_angle + value_angular;


    //Topology component calculation
    if( use_topology ){
      int temp_value_metacube_angles = 0 ; 
      value_topology = m_view_evaluator.inverseRayCast_simple(view, meta_holes, temp_value_metacube_angles ); 
      value_topology = value_topology / meta_holes.size() ; 
      //value_topology = value_topology * 0.25 ; // coefficient for topology
      ROS_INFO_COND(_sys_params.verbose_planning, "TOPOLOGY VALUE of  %d is %f", i, value_topology);

    }

    //Coordination component calculation
    if (m_team_size >= 2 ){

      coordination_value = 2.0 ; 


      if( (use_replanning == false) || m_coordinator.isViewAssigned(view_vector) ){

        if(use_repulsion_max == false){
          repulsion_force = _sys_params.distance_max;
          ROS_INFO_COND( _sys_params.verbose_planning  , "Distance max is %f", repulsion_force) ; 
        }
        else{
          repulsion_force = m_sensor_model.p_ray_length_; 
        }

        coordination_value = m_coordinator.evaluate_view( view_vector, m_team_goal, repulsion_force ) ; // higher the value the less good it is (because higher overlap)
        ROS_INFO_COND( _sys_params.verbose_planning  , "Coordination value out is %f", coordination_value) ; 

      }

    }


    total_value = _sys_params.image_component_weight * image_values[i] + _sys_params.metrics_component_weight * value_angular ; 
    if(use_topology){
      total_value = _sys_params.topology_component_weight * value_topology + total_value ; 
    }
    if(use_coordination){
      ROS_INFO_COND( _sys_params.verbose_planning , "COORDINATION VALUE of  %d is %f", i, (1.0 - coordination_value));
      total_value = _sys_params.coordination_component_weight * (1.0 - coordination_value) + total_value ; // we (1- x) because we look for the max value of the views, so max value should be more dispersed

      float randomness_coord = 0 ; 
      //randomness to prevent conflict

      // if( coordination_value > (0.5 /) ){
      //   std::random_device rd;
      //   std::mt19937 gen(rd());                    // Mersenne Twister RNG
      //   std::uniform_real_distribution<float> dist_rand(0.0f, 0.1f);
      //   randomness_coord = dist_rand(gen);

      // }


      total_value = total_value + randomness_coord ; 
    }
    ROS_INFO_COND(_sys_params.verbose_planning, "TOTAL VALUE OF  %d is %f", i, total_value);
    
    values_views.push_back(total_value);
    if( total_value > max_value_nbv){
      index_of_nbv = i;
      max_value_nbv = total_value;

    }

    // break ; 
    nb_views++;

  }


  //select views
  ros::Time total_view_evaluation_end = ros::Time::now();
  ros::Duration duration_total = total_view_evaluation_start - total_view_evaluation_end ; 
  ROS_INFO_COND(_sys_params.timer, "[NBV_Selector][Evaluate View] Total %.4f s", duration_total.toSec());

  ROS_INFO_COND(_sys_params.verbose_planning, "[NBV_Selector][Measure] Image average value %.4f ", total_image/nb_views);
  ROS_INFO_COND(_sys_params.verbose_planning, "[NBV_Selector][Measure] Dist/angle average value %.4f ", total_dist_angle/nb_views);



  if (views.size() > 0 && index_of_nbv !=-1){
    ROS_INFO_COND(_sys_params.verbose, "UPDATING CURRENT GOAL ");
    counter_goal = counter_goal +1 ; 
    ROS_INFO("NEW GOAL %d OF ROBOT %d IS VIEW %d", counter_goal, m_team_id, index_of_nbv);
    ROS_INFO(" %f %f %f ", views[index_of_nbv].x , views[index_of_nbv].y , views[index_of_nbv].z );
    for (const auto& [key, value] : m_team_goal) {

      ROS_INFO_COND(_sys_params.verbose_planning, "Stored goal for robot %d is",key) ;
      ROS_INFO_COND(_sys_params.verbose_planning,  "%f %f %f ", value.x, value.y, value.z);

    }

    
    ROS_INFO_COND(_sys_params.verbose, "CURRENT POS IS %f %f %f", m_current_pos.x, m_current_pos.y, m_current_pos.z);

    views_history.push_back( views[index_of_nbv]) ;
    Eigen::Vector3d origin_frontier_view = Eigen::Vector3d(views[index_of_nbv].o_x , views[index_of_nbv].o_y, views[index_of_nbv].o_z );
    frontiers_history_set.insert( origin_frontier_view ) ; 


    //Extract frontiers that should be seen by the view
    ROS_INFO_COND(_sys_params.verbose, "INACESS FRONTIERS NUMBER IS ", inacess_frontiers.size());
    extract_reached_frontiers(views[index_of_nbv]);
    ROS_INFO_COND(_sys_params.verbose, "INACESS FRONTIERS NUMBER IS ", inacess_frontiers.size());

    //Correct orientation of the goal
    ViewCandidate goal_corrected = views[index_of_nbv];
    float temp_number = goal_corrected.z ; 
    goal_corrected.z = goal_corrected.o_z ; 
    findOrientation(goal_corrected);
    goal_corrected.z = temp_number ; 

    ROS_INFO_COND(_sys_params.verbose, "CHECKS INTEGRITY GOAL ");
    integrity_view = checkIntegrityView(goal_corrected);

    //m_current_goal = views[index_of_nbv];
    m_current_goal = goal_corrected;



    //set replanning status value
    if(use_replanning){
      if (m_replanning_status[m_team_id] >= 2)
      {
        if(m_replanning_status[m_team_id] < m_forced_allocation_max_turns +1 ){ // 

          m_replanning_status[m_team_id] = m_replanning_status[m_team_id] +1 ;
        
        }
        else {
        m_replanning_status[m_team_id] = 0; 
        }      

      }
    }
    
  }
  else{
        ROS_INFO_COND(_sys_params.verbose, "[Integrity] NO VIEW FOUND ");
  }




}

void NBV_Selector::extract_reached_frontiers(const ViewCandidate& next_best_view){

  if( remove_inacess_frontiers == false ){
    return;
  }
  int temp_nb_frontiers_visible = -1; 
  m_view_evaluator.inverseRayCast( next_best_view, frontiers_set, temp_nb_frontiers_visible, inacess_frontiers_cache, true ); 
}

void NBV_Selector::next_best_view_closest_frontier(){
  ROS_INFO_COND(_sys_params.verbose, "CLOSEST FRONTIER MODE");
  
  // ROS_INFO_COND(_sys_params.verbose, "EMPTY SPACE");

  //evaluate views
  std::vector<float> values_views;
  float min_value_nbv = MAXFLOAT ; 
  float temp_value = MAXFLOAT ; 
  int index_of_nbv =-1;
  m_current_goal = m_current_pos;
  ROS_INFO_COND(_sys_params.verbose, "EXAMINING %d", views.size());
  for(int i=0;i< views.size();i++){

    ViewCandidate view = views[i] ; 
    Eigen::Vector3d view_vector = Eigen::Vector3d( view.x, view.y, view.z);
    Eigen::Vector3d current_pos_vector( m_current_pos.x ,m_current_pos.y , m_current_pos.z );

    temp_value = (view_vector - current_pos_vector).norm();
    if(temp_value < min_value_nbv){
      min_value_nbv = temp_value;
      index_of_nbv = i;
    }
    

  }



  if (views.size() > 0){
    ROS_INFO_COND(_sys_params.verbose, "UPDATING CURRENT GOAL ");
    ROS_INFO_COND(_sys_params.verbose, "NEW GOAL IS VIEW %d", index_of_nbv);
    ROS_INFO_COND(_sys_params.verbose, " %f %f %f ", views[index_of_nbv].x , views[index_of_nbv].y , views[index_of_nbv].z );
    ROS_INFO_COND(_sys_params.verbose, "CURRENT POS IS %f %f %f", m_current_pos.x, m_current_pos.y, m_current_pos.z);

    views_history.push_back( views[index_of_nbv]) ;

    //Correct orientation of the goal
    ViewCandidate goal_corrected = views[index_of_nbv];
    float temp_number = goal_corrected.z ; 
    goal_corrected.z = goal_corrected.o_z ; 
    findOrientation(goal_corrected);
    goal_corrected.z = temp_number ; 

    //Extract frontiers that should be seen by the view
    extract_reached_frontiers(views[index_of_nbv]);

    //m_current_goal = views[index_of_nbv];
    m_current_goal = goal_corrected;

    
  }

}


void NBV_Selector::next_best_view_count_frontier(){
  ROS_INFO_COND(_sys_params.verbose, "COUNT FRONTIER MODE");
  
  // ROS_INFO_COND(_sys_params.verbose, "EMPTY SPACE");

  //evaluate views
  m_current_goal = m_current_pos;

  int temp_nb_frontiers_visible = -1;
  float max_value_nbv = -1 ; 
  int index_of_nbv =-1;


  ROS_INFO_COND(_sys_params.verbose, "EXAMINING %d", views.size());

  for(int i=0;i< views.size();i++){

    ViewCandidate view = views[i] ; 
    Eigen::Vector3d view_vector = Eigen::Vector3d( view.x, view.y, view.z);
    Eigen::Vector3d current_pos_vector( m_current_pos.x ,m_current_pos.y , m_current_pos.z );

    m_view_evaluator.inverseRayCast_no_distance(view, frontiers_set, temp_nb_frontiers_visible, inacess_frontiers, false ); 


    if(temp_nb_frontiers_visible > max_value_nbv){
      max_value_nbv = temp_nb_frontiers_visible ; 
      index_of_nbv = i;
    }
    

  }



  if (views.size() > 0){
    ROS_INFO_COND(_sys_params.verbose, "UPDATING CURRENT GOAL ");
    ROS_INFO_COND(_sys_params.verbose, "NEW GOAL IS VIEW %d", index_of_nbv);
    ROS_INFO_COND(_sys_params.verbose, " %f %f %f ", views[index_of_nbv].x , views[index_of_nbv].y , views[index_of_nbv].z );
    ROS_INFO_COND(_sys_params.verbose, "CURRENT POS IS %f %f %f", m_current_pos.x, m_current_pos.y, m_current_pos.z);

    views_history.push_back( views[index_of_nbv]) ;

    //Correct orientation of the goal
    ViewCandidate goal_corrected = views[index_of_nbv];
    float temp_number = goal_corrected.z ; 
    goal_corrected.z = goal_corrected.o_z ; 
    findOrientation(goal_corrected);
    goal_corrected.z = temp_number ; 

    //Extract frontiers that should be seen by the view
    extract_reached_frontiers(views[index_of_nbv]);

    //m_current_goal = views[index_of_nbv];
    m_current_goal = goal_corrected;
    
  }

}

void::NBV_Selector::next_best_view_velocity(){

  //to implement
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "nbv_selector_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");  

    ///////////////Robot team initialization
    int team_id = 1;
    std::vector<int> robot_team ; 
    robot_team.push_back(team_id);

    ///////////////

    NBV_Selector nbv_selector = NBV_Selector(nh, nh_private, team_id,  robot_team);
    ros::spin();
    return 0;
}



