// utils.h
#pragma once  // or use include guards

#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <vector>
#include <sstream>
// Function declarations

struct ViewCandidate
{
    //position
    float x;
    float y;
    float z;

    //orientation
    double q_x ;
    double q_y;
    double q_z;
    double q_w;

    //origin of the generation
    float o_x;
    float o_y;
    float o_z;

    // ViewCandidate(int x, int y, int z, float qx, float qy, float qz, float qw, int ox, int oy, int oz)
    //     : x(x), y(y), z(z), q_x(qx), q_y(qy), q_z(qz), q_w(qw), o_x(ox), o_y(oy), o_z(oz) {}

};

struct BoundingBox
{
    //MAP PARAMETERS
    float x_max;
    float x_min;

    float y_max;
    float y_min;

    float z_max;
    float z_min;
    
};

struct QualityArea
{
    BoundingBox bounding_box;
    float quality;
};


bool verify_angle(const Eigen::Vector3d& frontier, const ViewCandidate& vc, float& angle, float angle_low, float angle_high);
bool isCorrectPos(const Eigen::Vector3d& pos,  const BoundingBox& bb);
bool isSafeGround(const Eigen::Vector3d& pos);
void printVectorOneLine(const std::vector<double>& vec);
bool checkIntegrityView(ViewCandidate current_view);
const char* voxelStateToString(unsigned char state);



bool verify_angle(const Eigen::Vector3d& frontier, const ViewCandidate& vc, float& angle, float angle_low, float angle_high){

    float reduction_angle = 3 ;
    //if find orientation failed to find a direction
    if( vc.q_x == 0 &&  vc.q_y == 0 && vc.q_z == 0 && vc.q_w == 0 ){
        return false;
    }

    double vertical_angle_rad = 0 ;
    double vertical_angle_deg = 0 ;
    Eigen::Vector3d next_voxel( vc.x, vc.y, vc.z );

    //Compute direction with frontier
    // Eigen::Vector3d direction_original = (frontier - next_voxel).normalized() ; // from view candidate, towards frontier
    // Eigen::Vector3d direction_proj( direction_original.x(), direction_original.y(), 0) ;
    //direction_proj.normalize() ;

    Eigen::Vector3d direction_original = (next_voxel - frontier).normalized() ; // from frontier towards view candidate
    Eigen::Vector3d direction_proj( direction_original.x(), direction_original.y(), 0 ) ;

    
    //Compute signed vertical angle
    //vertical_angle_rad = std::atan2(direction_original.z(), direction_original.head<2>().norm());
    vertical_angle_rad = std::atan2(direction_original.z(), direction_proj.norm()); //same as last line
    vertical_angle_deg = vertical_angle_rad * (180.0 / M_PI);

    // ROS_INFO("Angle found is function is  %f", vertical_angle_deg);
    angle = vertical_angle_deg;
    if( (vertical_angle_deg < angle_high - reduction_angle ) && (vertical_angle_deg > angle_low + reduction_angle) ){
        angle = vertical_angle_deg;
        return true;
    }

    return false;

}

bool isCorrectPos(const Eigen::Vector3d& pos, const BoundingBox& bb){



    if ( pos.z() >= bb.z_max || pos.z() <= bb.z_min ){
        return false;
    }

    if ( pos.y() >= bb.y_max || pos.y() <= bb.y_min ){
        return false;
    }

    if ( pos.x() >= bb.x_max || pos.x() <= bb.x_min ){
        return false;
    }

    return true ; 

  
}

bool get_quality_target_for_frontier(const std::vector<QualityArea>& qa, const Eigen::Vector3d& frontier, float& distance_min, float& distance_max){

    float temp_min = 0 ; 
    float temp_max = 0 ;
    float quality_threshold = 100 ;

    for(int i = 0 ; i< qa.size(); i++){

        // ROS_WARN("Evaluation quality %f ", qa[i].quality); 

        if( isCorrectPos(frontier, qa[i].bounding_box ) ){

            if(quality_threshold > qa[i].quality){

                quality_threshold = qa[i].quality ; 
            }
        }


    }

    if(quality_threshold == 100){
        return false ; 
    }

    temp_min = quality_threshold - 0.5 ; 
    temp_max = quality_threshold + 0.5 ;

    distance_min = temp_min ; 
    distance_max = temp_max ; 
    return true;


}

bool isSafeGround(const Eigen::Vector3d& pos){
    if (pos.z() <= 0.5 ){
        return false; 
    }
    return true;
}



void printVectorOneLine(const std::vector<int>& vec) {
    std::stringstream ss;
    ss << "[ ";
    for (int val : vec) {
        ss << val << " ";
    }
    ss << "]";
    ROS_INFO_STREAM("Vector: " << ss.str());
}


bool checkIntegrityView(ViewCandidate current_view){

    if (current_view.x < 1e-3 && current_view.y < 1e-3  && current_view.z < 1e-3 ){

        ROS_INFO("[Integrity] View %d  is 0 ");
        return false;
    }


    if ( std::isnan(current_view.x)  ||  std::isnan(current_view.y) || std::isnan(current_view.z)  ){

        ROS_INFO("[Integrity] View %d position  is nan ");
        return false;
    }

    if ( std::isnan(current_view.q_x)  ||  std::isnan(current_view.q_y) || std::isnan(current_view.q_z) || std::isnan(current_view.q_w)  ){

        ROS_INFO("[Integrity] View %d orientation is nan ");
        return false;
    }
    
    return true;



}

//TO VERIFY ! 
double computeSphereIntersection(float radius1, float radius2, float dist_center){
    if(radius1 <= 0 || radius2 <= 0  || dist_center < 0 ){
        ROS_WARN("MASSIVE ERROR IN SPHERE INTERSECTION");
        exit(1); 
    }
    if( dist_center >= (radius1 + radius2)){
        return 0 ; 
    }
    float min_radius = -1 ; 
    if(dist_center == 0 || (dist_center <= abs(radius1 - radius2) ) ){
        min_radius = std::min( radius1, radius2) ; 
        return (4.0/3.0) * M_PI * pow( min_radius, 3 ) ; 
    }
    double result = -1 ; 
    result = (M_PI / (12.0 * dist_center) ) ;
    result = result * pow( ( radius1 + radius2 - dist_center), 2 ) ;
    result = result * ( pow(dist_center,2) + ( 2.0 * dist_center *  (radius1 + radius2) ) - (3.0 * (radius1 - radius2) * (radius1 - radius2) ) ) ;
    return result ; 
}

const char* voxelStateToString(unsigned char state) {
    switch (state) {
        case 0: return "OCCUPIED";
        case 1: return "FREE";
        case 2: return "UNKNOWN";
        case 3: return "UNSURE_OCC";
        case 4: return "UNSURE_FREE";
        default: return "INVALID_STATE";
    }
}

struct Vector3dHash {
    std::size_t operator()(const Eigen::Vector3d& v) const {
        std::size_t h1 = std::hash<double>{}(v.x());
        std::size_t h2 = std::hash<double>{}(v.y());
        std::size_t h3 = std::hash<double>{}(v.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);  // Combine hashes
    }
};

struct Vector3dEqual {
    bool operator()(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
        return a.isApprox(b);  // Use Eigen's approximate equality
    }
};

struct MetaCube
{
    //Eigen::Vector3d coordinates; //center of the cube
    int knowledge_status; // 0 is unknown, 1 is free, 2 is occupied, 3 is containing frontiers, 4 is meta frontier
    static float size ; //size of a meta cube



    MetaCube(int status = 0) : knowledge_status(status) {}


    static void getCorners(const Eigen::Vector3d& coord, std::array<Eigen::Vector3d, 8>& corners){
        Eigen::Vector3d center = getCenter(coord); 
        corners[0] = Eigen::Vector3d( center.x() + (MetaCube::size/ 2) , center.y() + (MetaCube::size/ 2) , center.z() + (MetaCube::size/ 2));
        corners[1] = Eigen::Vector3d( center.x() - (MetaCube::size/ 2) , center.y() - (MetaCube::size/ 2) , center.z() + (MetaCube::size/ 2));
        corners[2] = Eigen::Vector3d( center.x() - (MetaCube::size/ 2) , center.y() + (MetaCube::size/ 2) , center.z() + (MetaCube::size/ 2));
        corners[3] = Eigen::Vector3d( center.x() + (MetaCube::size/ 2) , center.y() - (MetaCube::size/ 2) , center.z() + (MetaCube::size/ 2));

        corners[4] = Eigen::Vector3d( center.x() + (MetaCube::size/ 2) , center.y() + (MetaCube::size/ 2) , center.z() - (MetaCube::size/ 2));
        corners[5] = Eigen::Vector3d( center.x() - (MetaCube::size/ 2) , center.y() - (MetaCube::size/ 2) , center.z() - (MetaCube::size/ 2));
        corners[6] = Eigen::Vector3d( center.x() - (MetaCube::size/ 2) , center.y() + (MetaCube::size/ 2) , center.z() - (MetaCube::size/ 2));
        corners[7] = Eigen::Vector3d( center.x() + (MetaCube::size/ 2) , center.y() - (MetaCube::size/ 2) , center.z() - (MetaCube::size/ 2));



    }

    static Eigen::Vector3d getCenter(const Eigen::Vector3d& coord){
        Eigen::Vector3d center;
        Eigen::Vector3d meta_cube;
        getMetaCube(coord, meta_cube); 
        center.x() = meta_cube.x() + MetaCube::size/ 2 ; 
        center.y() = meta_cube.y() + MetaCube::size/ 2 ;
        center.z() = meta_cube.z() + MetaCube::size/ 2 ; 

        return center; 

    }

    static void getMetaCube(const Eigen::Vector3d& coord_3d, Eigen::Vector3d& meta_cube) {

        meta_cube.x() = (std::floor(coord_3d.x() / MetaCube::size));
        meta_cube.y() = (std::floor(coord_3d.y() / MetaCube::size));
        meta_cube.z() = (std::floor(coord_3d.z() / MetaCube::size));
    }

    bool isInside(const Eigen::Vector3d& pos){
        return false;
    }

};

float MetaCube::size = 0.0f ; 