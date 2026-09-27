#pragma once
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <cstdlib>
#include <voxblox_map/voxblox_map.h>
// #include <planning/modules/utils.h>
#include "regularized_k_means.h"
#include "Hungarian.h"
#include <random>
#include "ros/ros.h"

class MultiRobotCoordinator
{
private:
    std::string m_drone_name ;
    int m_team_size;
    int m_team_id;
    std::vector<int> m_team;

    //std::unordered_map<int, ViewCandidate> m_team_pos;
    //std::unordered_map<int, ViewCandidate> m_team_goal;
    std::unordered_map<int, int> assigned_views ;
    std::vector<Eigen::Vector3d> balanced_clusters_centroids ;
    std::vector<std::vector<double>> distance_matrix; 
    std::unordered_map<int, Eigen::Vector3d> assigned_centroids ;

    std::unordered_map<int, float> drones_ranges ;

    double assignment_cost_ ; 


    std::vector<std::vector<double>> views_ ;

    //std::vector<int> replanning_status_ ; 

    //
    std::string m_dispersion_method;
    bool m_verbose_multi ; 


public:
    MultiRobotCoordinator(const std::string& drone_name, int team_size, int team_id, const std::string& dispersion_method, bool verbose_multi);
    MultiRobotCoordinator(){};

    ~MultiRobotCoordinator();

    float compute_dispersion_value(float distance_robot_to_view, float robot_1_range, float robot_2_range );

    double evaluate_allocation(const std::unordered_map<int, ViewCandidate>& m_team_pos, const std::vector<ViewCandidate>& views); 
    void balance_clustering(const std::unordered_map<int, ViewCandidate>& m_team_pos, const std::vector<ViewCandidate>& views) ;

    void update_centroid_distance_matrix(const std::unordered_map<int, ViewCandidate>& m_team_pos);

    void assign_centroids_to_drones(); 
    void nearest_neighbor_assignment();
    void hungarian_algorithm_assignment(); 


    float evaluate_view(const Eigen::Vector3d& view_vector, const std::unordered_map<int, ViewCandidate>& m_team_goal, float repulsion_force);
    bool isViewAssigned(const Eigen::Vector3d& view);

    double get_assignment_cost(){ return assignment_cost_ ; }



    const std::unordered_map<int, int>& get_assigned_views() const { return assigned_views ; } ; 
    const std::unordered_map<int, Eigen::Vector3d>& get_assigned_centroids() const { return assigned_centroids ; } ;
    const std::vector<Eigen::Vector3d>& get_centroids() const { return balanced_clusters_centroids ; } ; 
    void set_assigned_centroids(const std::vector<Eigen::Vector3d>& external_plan){ 
        
        assigned_centroids.clear();
        for(int i=0;i<external_plan.size() ; i++){

            assigned_centroids[i] = external_plan[i] ; 
        }
        
    } ; 
    //std::vector<int> get_replanning_status(){ return replanning_status_ ; } ; 

    bool isReconfigurationPossible(std::vector<int> replanning_status); 
    




 
};

MultiRobotCoordinator::MultiRobotCoordinator(const std::string& drone_name, int team_size, int team_id, const std::string& dispersion_method, bool verbose_multi)
{
    m_drone_name = drone_name;
    m_team_id = team_id ; 
    m_team_size = team_size;
    m_dispersion_method = dispersion_method; 
    m_verbose_multi = verbose_multi ; 
    assignment_cost_  =-1; 

    for(int i=0; i<m_team_size;i++){
        m_team.push_back(i);

        distance_matrix.push_back( std::vector<double>( team_size,0.0) ) ;

        //replanning_status_.push_back( 0 ); 
    }

    
}



MultiRobotCoordinator::~MultiRobotCoordinator()
{
}


double MultiRobotCoordinator::evaluate_allocation(const std::unordered_map<int, ViewCandidate>& m_team_pos, const std::vector<ViewCandidate>& views){

    ViewCandidate view ; 
    Eigen::Vector3d robot_pos; 
    int assigned_robot_id = -1;
    float distance_view_robot ;
    float temp_distance ;
    double result = 0;
    float opt_assignment = (views.size() / (1.0 * m_team_size)) ; 


    for(int j : m_team){
                assigned_views[j] = 0 ; 
    }


    for(int i=0;i< views.size();i++){

        view = views[i] ; 
        Eigen::Vector3d view_vector = Eigen::Vector3d( view.x, view.y, view.z);

        distance_view_robot = MAXFLOAT;
        temp_distance = MAXFLOAT;
        assigned_robot_id = -1;

        for(int j : m_team){

            robot_pos = Eigen::Vector3d( m_team_pos.at(j).x,  m_team_pos.at(j).y, m_team_pos.at(j).z) ; 
            temp_distance = (robot_pos - view_vector).norm();
            if( temp_distance < distance_view_robot){
                assigned_robot_id = j ;
                distance_view_robot = temp_distance ; 
            }

        }

        assigned_views[ assigned_robot_id ] = assigned_views[ assigned_robot_id ] + 1 ; 

    }


    for(int j : m_team){
        auto diff =( assigned_views[j] - opt_assignment )  ; 
        result = result + diff * diff ; 
    }
    return result; 
}

void MultiRobotCoordinator::balance_clustering(const std::unordered_map<int, ViewCandidate>& m_team_pos, const std::vector<ViewCandidate>& views){

    for(int i=0; i< views.size(); i++){

        std::vector<double> temp_view{ views[i].x,  views[i].y,  views[i].z }; 
        views_.push_back( temp_view ) ; 


    }

    RegularizedKMeans rkmeans = RegularizedKMeans(views_, m_team_size);
    rkmeans.SolveHard();

    balanced_clusters_centroids.clear(); 

    for(int i=0 ; i< rkmeans.cluster_centers().size() ; i++){

        balanced_clusters_centroids.push_back( Eigen::Vector3d( rkmeans.cluster_centers().at(i).at(0) , rkmeans.cluster_centers().at(i).at(1)  ,  rkmeans.cluster_centers().at(i).at(2)) );
    }


}

void MultiRobotCoordinator::update_centroid_distance_matrix(const std::unordered_map<int, ViewCandidate>& m_team_pos){
    Eigen::Vector3d robot_pos; 
    Eigen::Vector3d current_centroid ; 

    if( balanced_clusters_centroids.size() != m_team_size){
        exit(1); 
    }


    //robot i centroid j 
    for(int i=0; i< m_team_size; i++){
                    
        robot_pos = Eigen::Vector3d( m_team_pos.at(i).x,  m_team_pos.at(i).y, m_team_pos.at(i).z) ; 

        for(int j=0; j< balanced_clusters_centroids.size(); j++){
            
            current_centroid = balanced_clusters_centroids[j];

            distance_matrix[i][j] = (robot_pos - current_centroid).norm() ; 

        }
    }
}

void MultiRobotCoordinator::nearest_neighbor_assignment(){
    double min_distance = MAXFLOAT;
    int index_min =-1; 
    std::set<int> assigned_centroids_index; 
    assignment_cost_ = 0 ; 

    //robot i centroid j
    for(int i=0; i< m_team_size; i++){

        assigned_centroids[i] = Eigen::Vector3d(0,0,0);

        min_distance = MAXFLOAT;
        index_min =-1; 
        for(int j=0; j< m_team_size; j++){

            if( (min_distance > distance_matrix[i][j]) && ( assigned_centroids_index.count(j) ==0 ) ){
                min_distance = distance_matrix[i][j] ; 
                index_min = j; 
            }

        }

        assigned_centroids[i] = balanced_clusters_centroids[index_min];
        assigned_centroids_index.insert(index_min);
        assignment_cost_  = assignment_cost_ + distance_matrix[i][index_min] ; 
    }

}

void MultiRobotCoordinator::hungarian_algorithm_assignment(){
    HungarianAlgorithm ha ; 
    std::vector<int> temporary_assignment;
    ha.Solve( distance_matrix, temporary_assignment); 
    int index_solution = -1;
    assignment_cost_ = 0 ; 

    for(int i =0; i< temporary_assignment.size(); i++){

        index_solution = temporary_assignment[i];
        assigned_centroids[i] = balanced_clusters_centroids[index_solution]; 
        assignment_cost_  = assignment_cost_ + distance_matrix[i][index_solution] ; 

    }




}



bool MultiRobotCoordinator::isReconfigurationPossible(std::vector<int> replanning_status){

    float result; 
    for(int i = 0; i< replanning_status.size(); i++){

        result = result + replanning_status[i];

    }

    if(result == 0 ){
        return true;
    }
    return false;

}

//misses m_sensor_model, m_team_goal
float MultiRobotCoordinator::evaluate_view(const Eigen::Vector3d& view_vector, const std::unordered_map<int, ViewCandidate>& m_team_goal, float repulsion_force){
    float coordination_value = 0 ; 
    float temp_coordination_value = 0 ;
    float temp_coord_distance = 0 ;  
    float angle_similarity = 0;
    int active_robots_coord = 0 ; 
    float max_distance_coord = 2 * repulsion_force; 
    for( int robot_id : m_team){
        if( m_team_id != robot_id){

            temp_coord_distance = max_distance_coord + 1 ; 
            if( m_team_goal.find(robot_id) != m_team_goal.end()){ //if goal of this other robot is available
            Eigen::Vector3d other_robot_goal_pos = Eigen::Vector3d( m_team_goal.at(robot_id).x, m_team_goal.at(robot_id).y, m_team_goal.at(robot_id).z);
            
            //tf2::Quaternion quat(m_team_goal[robot_id].q_x, m_team_goal[robot_id].q_y, m_team_goal[robot_id].q_z, m_team_goal[robot_id].q_w);
            //Eigen::Vector3d other_robot_goal_orientation =   tf2::Matrix3x3(quat) * tf2::Vector3(1, 0, 0) ;

            temp_coord_distance = (view_vector - other_robot_goal_pos).norm(); //compute the distance between the goal
            angle_similarity = 1 ; //to change
     

            }

            

            if( temp_coord_distance > max_distance_coord ){ //robot is too far so it has no impact on coordination
                temp_coordination_value = 0 ;
                ROS_INFO_COND( m_verbose_multi," Robot %d is not close", robot_id);
            }
            else{
                //strong decrease exponential method
                //temp_coordination_value = exp( - temp_coord_distance +2 * m_sensor_model.p_ray_length_ ) -1 ; 
                //temp_coordination_value = temp_coordination_value / (exp( 2 * m_sensor_model.p_ray_length_ ) - 1 ); // normalization

                //slow decrease exponential method
                // temp_coordination_value = exp(  temp_coord_distance -2 * m_sensor_model.p_ray_length_ ) -1 ; 
                // temp_coordination_value = temp_coordination_value / (exp( - 2 * m_sensor_model.p_ray_length_ ) - 1 ); // normalization

                // //common to both
                // temp_coordination_value = temp_coordination_value * angle_similarity ; //heading similarity

                //sphere intersection
                temp_coordination_value = computeSphereIntersection( repulsion_force , repulsion_force , temp_coord_distance); 
                temp_coordination_value = temp_coordination_value / ( computeSphereIntersection( repulsion_force , repulsion_force , 0.0)  ) ; 

                ROS_INFO_COND( m_verbose_multi,"Temp intersection val for robot %d is %f", robot_id, temp_coordination_value); 
                active_robots_coord ++ ; 
            }


            coordination_value = coordination_value + temp_coordination_value ; //here the highest value means the closest to other robots

        }

    }

    if( active_robots_coord > 0 ){
        coordination_value = coordination_value / active_robots_coord ;  
        ROS_INFO_COND( m_verbose_multi,"Active robots in vicinity : %d", active_robots_coord); 

    }
    else{
        coordination_value = 0 ; 
        ROS_INFO_COND( m_verbose_multi,"No robots in close proximity"); 
    }

    ROS_INFO_COND( m_verbose_multi,"Evaluating coordination %f", coordination_value);
    return coordination_value;
}

//check the assignment of a view based on previously computed/received centroids
bool MultiRobotCoordinator::isViewAssigned(const Eigen::Vector3d& view){
    Eigen::Vector3d current_centroid; 
    float distance_to_drone = MAXFLOAT;
    float temp_distance = -1;
    int ind_min = -1;

    for(int i = 0 ; i< m_team_size; i++){

        current_centroid = assigned_centroids[i];
        temp_distance = (view - current_centroid).norm();
        
        if(distance_to_drone > temp_distance){

            distance_to_drone = temp_distance;
            ind_min = i;
        }

    }

    //if view evaluated is closer to the centroid assigned to this robot, view is assigned
    if(ind_min == m_team_id){
        return true;
    }
    return false; 

}



