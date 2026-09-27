#pragma once
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <cstdlib>
#include <voxblox_map/voxblox_map.h>
// #include <planning/modules/utils.h>
#include <random>

struct ViewAngle
{
    float current_vert_angle ;
    float vert_angle_diff ; 
    float goal_vert_rotation; 
    float goal_vert_angle ; 
};


class ViewGenerator
{
private:
    /* data */
    std::string m_method_type;
    std::vector<ViewCandidate> view_candidates;
    std::vector<ViewCandidate> rejected_view_candidates;
    float m_distance_max; // max sampling distance from the frontiers
    float m_distance_min; // min sampling distance from the frontiers
    float m_angle_low ; // vertical angle below the drone
    float m_angle_high; // vertical angle above the drone 
    voxblox_map::VoxbloxMap m_map;
    Eigen::Vector3d c_neighbor_voxels_[26];
    std::vector<Eigen::Vector3d> rejected_frontiers; 

    std::default_random_engine rng_;
    std::normal_distribution<double> distribution_;

    std::vector<QualityArea> m_qa ; 


    bool m_verbose;

    bool m_forced_safety ;
    int max_sampling;
    float robot_radius_ ; 
    BoundingBox m_bb; 

public:
    ViewGenerator(
        const std::string& method_name,
        float distance_min,
        float distance_max,
        const voxblox_map::VoxbloxMap& map,
        float robot_radius,
        float angle_low,
        float angle_high,
        const BoundingBox& bb,
        bool forced_safety,
        const std::vector<QualityArea>& qa, 
        bool verbose
    );
    ViewGenerator(){};
    ~ViewGenerator(){};

    void generateViews(const std::vector<Eigen::Vector3d>& frontiers_set);
    void generateViews_sphere(const std::vector<Eigen::Vector3d>& frontiers_set);
    // void generateViews_echo(std::vector<Eigen::Vector3d> frontiers_set);
    void generateViews_normals(const std::vector<Eigen::Vector3d>& frontiers_set);
    bool generateview_normal(const Eigen::Vector3d& frontier, ViewAngle& view_angle, ViewCandidate& vc, float noise_value, Eigen::Vector3d& gradient_param);

    void generateViews_gradients_ESDF(const std::vector<Eigen::Vector3d>& frontiers_set);


    Eigen::Vector3d computeGradient(const Eigen::Vector3d& voxel);
    Eigen::Vector3d computeGradient_26(const Eigen::Vector3d& voxel);
    Eigen::Vector3d computeGradient_Sobel(const Eigen::Vector3d& voxel);

    Eigen::Vector3d generateNoisePosition(const Eigen::Vector3d& current_pos, const Eigen::Vector3d& frontier, float distance, float noise_value );

    

    std::vector<ViewCandidate> getViewCandidates(){ return view_candidates; }; // TO TRANSFORM BY REFERENCE
    std::vector<Eigen::Vector3d> getRejectedFrontiers(){ return rejected_frontiers; } // TO TRANSFORM BY REFERENCE
    std::vector<ViewCandidate> getRejectedViews(){ return rejected_view_candidates;} // TO TRANSFORM BY REFERENCE
    
    std::string getMethod_type(){ return m_method_type; } ;
    bool isSafeView(const Eigen::Vector3d& voxel);
    bool isSafeView_Forced(const Eigen::Vector3d& voxel);
    bool isSafeView_Wrapper(const Eigen::Vector3d& voxel);



    

    //std::string method_type;
    
};



void findOrientation(ViewCandidate& vc);
