#include "planning/modules/ViewGenerator.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include "ros/ros.h"
#include <algorithm>



ViewGenerator::ViewGenerator(
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
){
    m_method_type = method_name;
    m_distance_max = distance_max;
    m_distance_min = distance_min;
    m_map = map;
    robot_radius_ = robot_radius ; 
    m_angle_low = angle_low ;
    m_angle_high = angle_high ; 
    m_bb = bb;
    m_qa = qa ; 

    m_verbose = verbose;

    rng_.seed(std::random_device{}());
    distribution_ = std::normal_distribution<double>(0.0, 0.01);


    m_forced_safety = forced_safety ; //Force views to be in free known space
    rejected_frontiers = std::vector<Eigen::Vector3d>();
    max_sampling = 1;
    if(distance_min > distance_max) {

        //throw exception
        throw std::string(" Distances values are incorrect. Can not initialize ViewGenerator");
    }

    float vs = m_map.getVoxelSize() ; 
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

void ViewGenerator::generateViews(const std::vector<Eigen::Vector3d>& frontiers_set ){
    if(m_method_type == "sphere"){

        ViewGenerator::generateViews_sphere( frontiers_set );
    }
    else if(m_method_type == "gradient"){
        //ViewGenerator::generateViews_gradients_ESDF( frontiers_set ) ; 
        ViewGenerator::generateViews_normals(frontiers_set);
    }
    else{
        throw std::invalid_argument("Methods not defined ! ");
    }
}


void ViewGenerator::generateViews_sphere(const std::vector<Eigen::Vector3d>& frontiers_set ){

    view_candidates.clear();
    float temp_distance_min = 1 ; 
    int count_while = 0 ; 
    int max_try_sampling = 1000; 
    float angle_low_radian = (m_angle_low * M_PI  /180) + (2 * M_PI); 
    float angle_high_radian = (m_angle_high * M_PI / 180); 

    float r = 0;
    float phi =0;
    float theta = 0;
    float vert_angle = 0;
    ROS_INFO_COND(m_verbose,"Bounding box is %f %f %f %f %f %f", m_bb.x_max, m_bb.x_min, m_bb.y_max, m_bb.y_min, m_bb.z_max, m_bb.z_min);
    ROS_INFO_COND(m_verbose," Vert angle should be between %f and %f", angle_low_radian, 2 * M_PI) ; 
    ROS_INFO_COND(m_verbose," Vert angle should be between %f and %f", 0.0 , angle_high_radian) ; 

    std::random_device rd;
    std::mt19937 gen;
    gen = std::mt19937(rd());  //

    std::uniform_real_distribution<float> dist_r;
    std::uniform_real_distribution<float> dist_angle;

    float distance_min_sphere = robot_radius_ * m_map.getVoxelSize();
    // Reset the distributions with proper ranges
    dist_r = std::uniform_real_distribution<float>(distance_min_sphere, m_distance_max);
    dist_angle = std::uniform_real_distribution<float>(0.0f, 2.0f * M_PI);

    float total =0; 

 
    for(int i=0; i< frontiers_set.size(); i++){

        int numberOfOccupiedVoxels = 0;
        int numberOfUnknownVoxels = 0;
        count_while = 0 ; 

        Eigen::Vector3d frontier = frontiers_set[i];
        float temp_x = frontier.x();
        float temp_y = frontier.y();
        float temp_z = frontier.z(); 

        ROS_INFO_COND(m_verbose,"Looking for view %d", i);

        for(int j=0; j< max_sampling; j++){

            float formula = -1 ;
            bool isFree = false;
            bool vert_angle_bool = false;
            float o_x = 0;
            float o_y = 0 ;
            float o_z = 0 ; 
            count_while = 0 ;

            while((isFree == false) && (count_while < max_try_sampling) && (vert_angle_bool == false)){
                ROS_INFO_THROTTLE(10, "Looking for an unoccupied view...");
                
                count_while = count_while +1 ; 
            
                // float r = ( (static_cast<float>(rand()) / RAND_MAX) * ( m_distance_max - temp_distance_min ) + temp_distance_min) ;
                // float theta = ( (static_cast<float>(rand()) / RAND_MAX)  * 2 * M_PI) ;
                // float phi = ( (static_cast<float>(rand()) / RAND_MAX) * 2 * M_PI) ;

                // Sampling
                r = dist_r(gen);
                theta = dist_angle(gen);
                phi = dist_angle(gen);

                vert_angle = theta - (M_PI / 2) ;
                vert_angle = fmod( vert_angle, 2 * M_PI) ;

                ROS_INFO_COND(m_verbose," Trying %f %f %f", r, theta, phi);


                if( ( (vert_angle > 0) && (vert_angle < angle_high_radian)) || ( (vert_angle > angle_low_radian) && (vert_angle < (2*M_PI))) ){
                    ROS_INFO_COND(m_verbose,"Test 1: %s", (vert_angle > 0) ? "true" : "false");
                    ROS_INFO_COND(m_verbose,"angle_high_radian is: %f", angle_high_radian);
                    ROS_INFO_COND(m_verbose,"Test 2: %s", (vert_angle < angle_high_radian) ? "true" : "false");
                    ROS_INFO_COND(m_verbose,"Test 3: %s", (vert_angle > angle_low_radian) ? "true" : "false");
                    ROS_INFO_COND(m_verbose,"Test 4: %s", (vert_angle < (2*M_PI)) ? "true" : "false");
                    vert_angle_bool = true; 
                    ROS_INFO_COND(m_verbose,"Vert angle is %f", vert_angle);
                    ROS_INFO_COND(m_verbose,"Vert angle is %f", vert_angle / M_PI * 180 );
                }

                //spherical coordinates
                o_x = temp_x + r * sin( theta ) * cos( phi ) ; 
                o_y = temp_y + r * sin( theta ) * sin ( phi );
                o_z = temp_z + r * cos( theta ) ;

                //cylindrical coordinates
                // float o_x = r * cos( theta ) ;
                // float o_y = r * sin( theta ) ;
                // float o_z = temp_z ? ;

                Eigen::Vector3d voxel  = Eigen::Vector3d( o_x, o_y, o_z);
                unsigned char current_state = m_map.getVoxelState_ESDF(voxel);
                //isFree = (current_state == voxblox_map::VoxbloxMap::FREE);
                // if (current_state == voxblox_map::VoxbloxMap::OCCUPIED)
                //     numberOfOccupiedVoxels++;
                isFree = isSafeView_Wrapper(voxel); // never converges

                // if( current_state != voxblox_map::VoxbloxMap::FREE ){
                //     isFree = false;
                // }

                // if (current_state == voxblox_map::VoxbloxMap::UNKNOWN){
                //     isFree = false;
                // }
                // else{
                //     isFree = isSafeView_Wrapper_Wrapper(voxel); // never converges
                // }
                //     numberOfUnknownVoxels++;

                if(isFree){
                    ROS_INFO_COND(m_verbose,"Generating views %f %f %f ", o_x, o_y, o_z );
                }
                //ROS_INFO_COND(m_verbose,"Generating views %f %f %f ", o_x, o_y, o_z );


                //formula = (o_x - temp_x ) * (o_x - temp_x ) + (o_y - temp_y ) * (o_y - temp_y ) + (o_z - temp_z ) * (o_z - temp_z ) ;
            }

            if( count_while > max_try_sampling){
                ROS_INFO_COND(m_verbose,"Couldn't find view in %d times", count_while);
                continue;
            }

            if(isFree == true && vert_angle_bool == true){

                ROS_INFO_COND(m_verbose,"Found view in %d times", count_while);
                Eigen::Vector3d voxel  = Eigen::Vector3d( o_x, o_y, o_z);
                unsigned char current_state = m_map.getVoxelState_ESDF(voxel);
                ROS_INFO_COND(m_verbose,"View is type %d", current_state);
    
    
    
                // ROS_INFO_COND(m_verbose,"While looking for an unoccupied view, found %i occupied and %i unknown voxels", numberOfOccupiedVoxels, numberOfUnknownVoxels);
    
                ViewCandidate vc = {o_x,o_y,o_z, 0,0,0,0, temp_x, temp_y, temp_z};
                
                findOrientation(vc);
    
                view_candidates.push_back( vc ) ;
                total = total +1;

            }
            

        


        }






    }


    ROS_INFO(" SUCCESS RATE VIEWS IS %f ",total);

}

void ViewGenerator::generateViews_gradients_ESDF(const std::vector<Eigen::Vector3d>& frontiers_set){
    view_candidates.clear();
    float step_size = 0.5 ; 
    //int nb_steps = m_distance_max / m_map.getVoxelSize() ;
    int nb_steps = m_distance_max / step_size ;
    Eigen::Vector3d gradient ; 
    Eigen::Vector3d last_voxel, next_voxel ; 
    float distance = 0 ;
    bool isSafeView_bool ; 
    double vertical_angle_rad = 0 ;
    double vertical_angle_deg = 0 ;


    for(int i=0; i< frontiers_set.size(); i++){


        Eigen::Vector3d frontier = frontiers_set[i];
        next_voxel = frontier ; 

        ROS_INFO_COND(m_verbose," Generating view for frontier %d ",i);
        for(int j=0;j<= nb_steps; j++ ){
            
            last_voxel = next_voxel ;
            //find gradient
            gradient = computeGradient(last_voxel) ; 

            //find corresponding voxel
            //m_map.getVoxelCenter_ESDF( &next_voxel, (gradient + last_voxel)) ;
            next_voxel = (last_voxel + gradient) ; 
            distance = (frontier - next_voxel).norm() * m_map.getVoxelSize(); 
            ROS_INFO_COND(m_verbose," Moved from %f to %f now at distance %f", m_map.getDistancePrecise_ESDF(last_voxel), m_map.getDistancePrecise_ESDF(next_voxel), distance );
            
            //Angle verification
            //Compute direction with frontier
            // Eigen::Vector3d direction_original = (frontier - next_voxel).normalized() ; 
            // Eigen::Vector3d direction_proj( direction_original.x(), direction_original.y(), 0) ;
            // direction_proj.normalize() ;

            // //Compute signed vertical angle
            // vertical_angle_rad = std::atan2(direction_original.z(), direction_original.head<2>().norm());
            // //double angle_rad_atan2 = std::atan2(direction_original.z(), direction_proj.norm()); //same as last line
            // vertical_angle_deg = vertical_angle_rad * (180.0 / M_PI);
            



            
            
            //isSafeView_bool = isSafeView(next_voxel);
            isSafeView_bool = true ; 
            if( (distance  >  (( m_distance_max + m_distance_min)/2)) && (isSafeView_bool) ){
                break; 
            } 


        }

        if(isSafeView_bool){

            ViewCandidate vc = { next_voxel.x() , next_voxel.y() , next_voxel.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
            findOrientation( vc );
            ROS_INFO_COND(m_verbose," Angle found is %f", vertical_angle_deg);
            
            view_candidates.push_back( vc );


        }
        else{
            ROS_INFO_COND(m_verbose,"Impossible to generate view for frontier %d ",i);

            
        }



        

    }



}



// void findOrientation(ViewCandidate& vc){

//     Eigen::Vector3d view_pos( vc.x, vc.y, vc.z ) ;
//     Eigen::Vector3d frontier_pos( vc.o_x, vc.o_y, vc.o_z ) ;

//     //Computes direction vector
//     Eigen::Vector3d direction =  frontier_pos - view_pos ; 
    
//     // Normalize the direction vector to get the forward vector
//     Eigen::Vector3d forward = direction.normalized();

//     float heading_angle =  atan2( forward.y(), forward.x() ) ;
//     float yaw = heading_angle ; 
//     // Compute the right vector as the cross product of up and forward

//     float pitch = asin( forward.z() ) ;
//     float roll = 0 ; 

//     float qx = sin(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0) - cos(roll/2.0) * sin(pitch/2.0) * sin(yaw/2.0) ;
//     float qy = cos(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0) + sin(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0) ;
//     float qz = cos(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0) - sin(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0) ;
//     float qw = cos(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0) ;

//     vc.q_x = qx;
//     vc.q_y = qy;
//     vc.q_z = qz;
//     vc.q_w = qw;


// }

// void findOrientation(ViewCandidate& vc){
//     // Convert the points to tf2::Vector3 for easier vector operations
//     tf2::Vector3 view_pos(vc.x, vc.y, vc.z);
//     tf2::Vector3 frontier_pos(vc.o_x, vc.o_y, vc.o_z);

//     // Compute the direction vector from point1 to point2
//     tf2::Vector3 direction = frontier_pos - view_pos;

//     if (direction.length2() <= 0.0) {
//         ROS_INFO_COND(m_verbose,"Direction vector is zero! Cannot normalize.");
//         return;
//     }
    
//     // Normalize the direction vector (to ensure it's a unit vector)
//     direction.normalize();

//     // The forward direction that "view_pos" should align with (now along the X-axis)
//     tf2::Vector3 forward(1.0, 0.0, 0.0); // Align with the X-axis

//     // Compute the axis of rotation (cross product of forward and direction)
//     tf2::Vector3 axis = forward.cross(direction);

//     if (axis.length2() <= 0.0) {
//         ROS_INFO_COND(m_verbose,"Rotation axis is zero! Possible parallel vectors.");
//         return;
//     }

//     axis.normalize();  // Normalize the axis

//     // Compute the angle of rotation (dot product gives cosine of the angle)
//     float dot = forward.dot(direction);
//     // Clamp the dot product to avoid precision errors in the acos function
//     dot = std::min(1.0f, std::max(-1.0f, dot));

//     // Calculate the angle between the vectors
//     float angle = acos(dot);

//     // Compute the quaternion representing the rotation
//     tf2::Quaternion rotation;
//     rotation.setRotation(axis, angle);

//     if( (std::isnan(rotation.x())) ||  (std::isnan(rotation.y())) || (std::isnan(rotation.z())) || (std::isnan(rotation.w()))  ){
        
//     }

//     vc.q_x = rotation.x();
//     vc.q_y = rotation.y();
//     vc.q_z = rotation.z();
//     vc.q_w = rotation.w();
// }

void findOrientation(ViewCandidate& vc) { // CHATGPT
    // Convert the points to tf2::Vector3 for easier vector operations
    tf2::Vector3 view_pos(vc.x, vc.y, vc.z);
    tf2::Vector3 frontier_pos(vc.o_x, vc.o_y, vc.o_z);

    // Compute the direction vector from view_pos to frontier_pos
    tf2::Vector3 direction = frontier_pos - view_pos;

    if (direction.length2() <= 0.0) {
        ROS_WARN("Direction vector is zero! Cannot normalize.");
        return;
    }
    
    direction.normalize(); // Ensure it's a unit vector

    // Define the forward vector (assuming object initially faces +X)
    tf2::Vector3 forward(1.0, 0.0, 0.0); // Default reference

    // Compute rotation axis using cross product
    tf2::Vector3 axis = forward.cross(direction);

    // Compute the dot product for the angle
    float dot = forward.dot(direction);
    dot = std::max(-1.0f, std::min(1.0f, dot)); // Prevent floating-point precision issues

    // Handle parallel vectors (cross product = 0)
    tf2::Quaternion rotation;
    if (axis.length2() <= 0.0) {
        ROS_WARN("Rotation axis is zero! Vectors are parallel.");

        // If vectors are identical (dot ≈ 1), set identity rotation
        if (dot > 0.9999f) {
            rotation = tf2::Quaternion(0, 0, 0, 1);
        }
        // If vectors are opposite (dot ≈ -1), apply a 180-degree rotation around Z or Y
        else {
            tf2::Vector3 perpendicular(0, 0, 1); // Stable perpendicular axis
            rotation.setRotation(perpendicular, M_PI);
        }
    } else {
        axis.normalize();
        float angle = acos(dot);
        rotation.setRotation(axis, angle);
    }

    // Prevent NaN values
    if (std::isnan(rotation.x()) || std::isnan(rotation.y()) ||
        std::isnan(rotation.z()) || std::isnan(rotation.w())) {
        ROS_WARN("Quaternion contains NaN! Resetting to identity.");
        ROS_ERROR("Quaternion contains NaN! Resetting to identity.");
        rotation = tf2::Quaternion(0, 0, 0, 1);
    }

    // Store the computed quaternion in ViewCandidate
    vc.q_x = rotation.x();
    vc.q_y = rotation.y();
    vc.q_z = rotation.z();
    vc.q_w = rotation.w();
}


void ViewGenerator::generateViews_normals(const std::vector<Eigen::Vector3d>& frontiers_set){

    double vertical_angle_rad = 0 ;
    double vertical_angle_deg = 0 ;
    bool generated = false;
    ViewAngle view_angle ; 
    float nb_rotation = 0 ; 
    float nb_sucess = 0 ; 
    float total = 0 ;
    Eigen::Vector3d gradient_param(0,0,0); 

    float gradient_failures = 0 ; 
    float no_space = 0;
    float noise_value = 0;
    float noise_saved_frontiers = 0;
    float total_noise = 0;
    float old_angle = 0 ;

    view_candidates.clear();
    rejected_frontiers.clear();
    rejected_view_candidates.clear();

    for(int i=0; i< frontiers_set.size(); i++){

        gradient_param = Eigen::Vector3d(0,0,0);

        ViewCandidate vc; 
        Eigen::Vector3d frontier = frontiers_set[i];
        ROS_INFO_COND(m_verbose," Generating view for frontier %d ",i);
        view_angle.goal_vert_rotation = 0; 
        generated = generateview_normal(frontier, view_angle, vc, 0, gradient_param);

        //if worked 
        if(generated){
            view_candidates.push_back( vc );
            // ROS_INFO_COND(m_verbose," %d Found view  %f %f %f ",i, vc.x, vc.y, vc.z);
            nb_sucess = nb_sucess +1 ; 
            continue;
        }

        if(generated == false && view_angle.vert_angle_diff == 0 ){
            ROS_INFO_COND(m_verbose," FAILURE : Gradient failed  %d ",i);
            old_angle = view_angle.current_vert_angle; 
            ROS_INFO_COND(m_verbose," current angle is  %f ",old_angle);


            // noise_value = 0;
            // while (noise_value < 60 && generated== false){

            //     noise_value = noise_value +30;
            //     // ROS_INFO_COND(m_verbose," Trying noise  %f ",noise_value);
            //     generated = generateview_normal(frontier, view_angle, vc, noise_value, gradient_param);
            //     // ROS_INFO_COND(m_verbose," new angle is  %f ",view_angle.current_vert_angle);

            //     if( view_angle.vert_angle_diff != 0 ){
            //         ROS_WARN("FATAL ERROR BULLSHIT");
            //         exit(1);
            //     }
            // }

            if(generated == false){
                // ROS_INFO_COND(m_verbose," Failed despite noise  %f ",noise_value);
                rejected_frontiers.push_back( frontier ) ;
                gradient_failures= gradient_failures +1;
                continue ; 
            }
            else{
                // ROS_INFO_COND(m_verbose," Survived with noise  %f ",noise_value);
                total_noise = total_noise + noise_value ;
                noise_saved_frontiers = noise_saved_frontiers +1;
                view_candidates.push_back( vc );
                // ROS_INFO_COND(m_verbose," %d Found view  %f %f %f ",i, vc.x, vc.y, vc.z);
                nb_sucess = nb_sucess +1 ; 

            }



        }
        
        ROS_INFO_COND(m_verbose," FAILURE : Rotation required  %d ",i);
        no_space = no_space +1;
        ROS_INFO_COND(m_verbose," ROTATION %d ",i);
        // search for new view with a corrected vertical angle
        view_angle.goal_vert_rotation = (1.0* view_angle.vert_angle_diff); 
        gradient_param = Eigen::Vector3d(0,0,0);
        generated = generateview_normal(frontier, view_angle, vc, 0, gradient_param);
        //if worked
        if(generated){
            view_candidates.push_back( vc );
            nb_rotation = nb_rotation +1 ;
            continue;
        }
        else{
            ROS_INFO_COND(m_verbose," Rotation failed  %d ",i);
        }

        //search for new view with a different horizontal angle
        //if didn't manage to find a solution
        rejected_frontiers.push_back( frontier ) ; 


        
    }
    total = (nb_rotation + nb_sucess) / frontiers_set.size() ; 
    ROS_INFO(" SUCCESS RATE VIEWS IS %f ",total);
    ROS_INFO(" REJECTED VIEWS SIZE IS %d ", rejected_view_candidates.size());
    if(frontiers_set.size()> 0 ){
        ROS_INFO(" Gradient failure %f rotation/noise required %f", gradient_failures/frontiers_set.size(), no_space/frontiers_set.size());
        ROS_INFO(" Survived with noise %f for average noise %f ", noise_saved_frontiers, total_noise/noise_saved_frontiers);
        ROS_INFO(" Noise succeeded in %f", noise_saved_frontiers/(noise_saved_frontiers/gradient_failures));
        // ROS_INFO("Rotation sucess is %f ", nb_rotation/no_space);

    }


    ROS_INFO_COND(m_verbose," Starting view integrity check");
    for(int i=0; i< view_candidates.size();i++ ){

        ViewCandidate current_view = view_candidates[i];
        checkIntegrityView(current_view);

    }



        
     

}

bool ViewGenerator::generateview_normal(const Eigen::Vector3d& frontier, ViewAngle& view_angle, ViewCandidate& vc, float noise_value, Eigen::Vector3d& gradient_param){
    ViewCandidate min_view ; 
    ViewCandidate mid_view;
    ViewCandidate max_view ;
    ViewCandidate rejected_view;
    
    Eigen::Vector3d close_point;
    Eigen::Vector3d mid_point;
    Eigen::Vector3d far_point;

    float angle_min = 0;
    float angle_mid = 0;
    float angle_max = 0;

    Eigen::Vector3d gradient ; 
    Eigen::Vector3d current_pos ;
    Eigen::Vector3d test_pos;
    Eigen::Vector3d vector_director ;  
    float distance = 0 ;
    float angle = -180 ; 
    bool isSafeView_bool = false; 
    bool isAngleok = true;

    //noise generation
    float noise_value_radian = noise_value * M_PI / 180.0;
    distribution_ = std::normal_distribution<double>(0.0, noise_value_radian);

    float rotation = view_angle.goal_vert_rotation ;
    //float goal_angle = view_angle.current_vert_angle - rotation ; 
    float goal_angle = view_angle.current_vert_angle + rotation ; 


    float rotation_angle_radian =  rotation * (M_PI / 180.0);  
    float goal_angle_radian = goal_angle * (M_PI / 180.0);  
    float rotation_cosinus = cos( rotation_angle_radian) ;
    float rotation_sinus = sin( rotation_angle_radian) ; 

    bool minimum = false ;
    bool mid = false ; 
    bool max_bool = false ; 

    //if multi_quality_area 
    float distance_max_obs  = m_distance_max ; 
    float distance_min_obs = m_distance_min ; 

    float temp_dist_min = 0 ;
    float temp_dist_max = 0 ; 

    if( m_qa.size() > 0  && get_quality_target_for_frontier(m_qa, frontier, temp_dist_min, temp_dist_max) ){

        ROS_WARN("Quality area : distance has been changed") ; 
        distance_max_obs = temp_dist_max ; 
        distance_min_obs = temp_dist_min ; 
        ROS_WARN("Quality area : %f", (distance_max_obs + distance_min_obs)/2.0 ) ; 
    }
    ROS_INFO_COND( m_verbose , "Distance max is %f", distance_max_obs) ; 
    ROS_INFO_COND( m_verbose, "Distance min is %f", distance_min_obs) ; 


    //end multi_quality_area

    //find gradient or take parameter
    if (gradient_param.norm() == 0 ){
        gradient = computeGradient_26(frontier) ; 
        gradient_param = gradient;
    }
    else{
        gradient = gradient_param; 
    }

    current_pos = frontier ;
    // ROS_INFO_COND(m_verbose," Gradient is %f %f %f ",gradient.x(), gradient.y(), gradient.z());
    // ROS_INFO_COND(m_verbose," Position is %f %f %f ",frontier.x(), frontier.y(), frontier.z());
    if( (gradient.x() == 0) && (gradient.y() == 0) && (gradient.z() ==0) ){
        ROS_INFO_COND(m_verbose," Gradient is 0 : view can not be found");
        view_angle.vert_angle_diff = 0 ;
        return false; 
    }


    //very important ! 
    gradient.normalize() ;

    //change of angle if necessary 
    if( rotation != 0){


        //VECTOR BASED ROTATION
        //Eigen::Vector3d temp_vect_calc( gradient.x(), gradient.y(), 0) ; 
        // Eigen::Vector3d temp_vect_calc( gradient.x(), gradient.y(), -gradient.z() ) ; 
        // temp_vect_calc.normalize();

        // Eigen::Vector3d axis_rotation = gradient.cross( temp_vect_calc);
        // axis_rotation.normalize();

        // Eigen::Matrix3d mat_rot = Eigen::AngleAxisd(rotation_angle_radian, axis_rotation).toRotationMatrix();

        // vector_director = mat_rot * gradient ; 





        //LATEST : 
        //TRIGONOMETRY BASED ROTATION
        float current_angle_radian = view_angle.current_vert_angle *  (M_PI / 180.0); 
        ROS_INFO_COND(m_verbose," Angle gradient is %f", view_angle.current_vert_angle);
        ROS_INFO_COND(m_verbose," Angle gradient is %f", current_angle_radian);
        //current_angle_radian = current_angle_radian + M_PI ;
        //float new_height = -tan(current_angle_radian + rotation_angle_radian) * sqrt( gradient.x() * gradient.x() + gradient.y() * gradient.y()) ;
        ROS_INFO_COND(m_verbose," Aimed new angle is %f", goal_angle);
        ROS_INFO_COND(m_verbose," Aimed new angle (radian) is %f", goal_angle_radian);
        //float new_height = tan(current_angle_radian ) * sqrt( gradient.x() * gradient.x() + gradient.y() * gradient.y()) ;
        float new_height = tan(goal_angle_radian ) * sqrt( gradient.x() * gradient.x() + gradient.y() * gradient.y()) ;

        vector_director = Eigen::Vector3d( gradient.x() , gradient.y(), new_height);
                                                                        

        ROS_INFO_COND(m_verbose," Rotated goal was %f", rotation);
        // ROS_INFO_COND(m_verbose," Rotated achieved was %f", );

        ROS_INFO_COND(m_verbose," Original gradient is %f %f %f ",gradient.x()  , gradient.y()  , gradient.z() );
        ROS_INFO_COND(m_verbose," Rotated gradient is %f %f %f ",vector_director.x(), vector_director.y(), vector_director.z());

        vector_director = vector_director * m_map.getVoxelSize();


    }
    else{
        vector_director = gradient ; 

        //UNSURE
        vector_director = vector_director * m_map.getVoxelSize();
        ROS_INFO_COND(m_verbose," Vector director is %f %f %f ",vector_director.x(), vector_director.y(), vector_director.z());

    }



    //ANGLE VERIFICATION
    ROS_INFO_COND(m_verbose," Angle should be between %f and %f", m_angle_low, m_angle_high);

    float vertical_view_angle = 0 ; 
    Eigen::Vector3d test_point = current_pos + 10 * vector_director; 
    ViewCandidate temp_view = { test_point.x() , test_point.y() , test_point.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
    findOrientation(temp_view) ; 
    verify_angle( frontier, temp_view, vertical_view_angle, m_angle_low, m_angle_high ) ; 

    if( rotation != 0){
        ROS_INFO_COND(m_verbose," New angle found is %f", vertical_view_angle);
    }
    else{
        ROS_INFO_COND(m_verbose," Original angle found is %f", vertical_view_angle);
    }

    view_angle.current_vert_angle = vertical_view_angle ; 
    float angle_tolerance = 3;
    if( vertical_view_angle > m_angle_high ){
        //view_angle.vert_angle_diff  = m_angle_high - angle_mid ;
        view_angle.vert_angle_diff  = (m_angle_high - angle_tolerance) - vertical_view_angle ;

        rejected_view_candidates.push_back(temp_view);
        ROS_INFO_COND(m_verbose," Angle rejected :  %f ! Too high ! ", vertical_view_angle);
        return false;
    }
    else if( vertical_view_angle < m_angle_low ){
        //view_angle.vert_angle_diff  = m_angle_low - angle_mid ;
        view_angle.vert_angle_diff  = (m_angle_low + angle_tolerance) - vertical_view_angle ;
        rejected_view_candidates.push_back(temp_view);
        ROS_INFO_COND(m_verbose," Angle rejected :  %f ! Too low ! ", vertical_view_angle);
        return false;
    }
    else {
        view_angle.vert_angle_diff  = 0;
        ROS_INFO_COND(m_verbose," Angle accepted :  %f ! ", vertical_view_angle);
    }
    


    //TO DO : test the uncommented line
    //current_pos = current_pos + ( (distance_min_obs / m_map.getVoxelSize() ) -1 ) * vector_director ; 

    //int number_it = (distance_max_obs + 0.2 - distance_min_obs) / ( m_map.getVoxelSize()) ; //UNSURE! !!!!!!!!!!!!
    //int number_it = (distance_max_obs + 0.2) / (vector_director.norm() * m_map.getVoxelSize()) ; 
    int number_it = (distance_max_obs + 0.2) / (vector_director.norm() ) ; 
    //go along the gradient direction
    for(int k =0 ; k< number_it; k++){
    //while( distance < distance_max_obs){

        current_pos = current_pos + (1 * vector_director) ;  
        //distance = ((current_pos - frontier).norm()) * m_map.getVoxelSize(); 
        distance = ((current_pos - frontier).norm()) ; 
        // ROS_INFO_COND(m_verbose," Distance is %f  until %f", distance, distance_max_obs);
        test_pos = current_pos;
        if( noise_value > 0 ){
            test_pos = generateNoisePosition(current_pos,frontier, distance, noise_value );
        }

        
        //if we are in the correct range 
        //compute is position safe 
        if(distance > (distance_min_obs - 0.2) ){ //start computing safe positions just before zone of interest
            isSafeView_bool = isSafeView_Wrapper(test_pos); 
            //isSafeView_bool = true; 
        }

        //closest point possible 
        if ((minimum == false) && (distance >  distance_min_obs) && (distance < (distance_min_obs + distance_max_obs)/2 ) && isSafeView_bool) {
            min_view = { test_pos.x() , test_pos.y() , test_pos.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
            // ROS_INFO_COND(m_verbose," View found min");
            minimum = true ; 
        }
        //closest point from the center
        else if( (mid == false) && (distance < distance_max_obs) && (distance > (distance_min_obs + distance_max_obs)/2 ) && isSafeView_bool ){
            mid_view = { test_pos.x() , test_pos.y() , test_pos.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
            // ROS_INFO_COND(m_verbose," View found mid");
            mid = true ; 
        }


    }
    if( isSafeView_bool ){
        max_view = { test_pos.x() , test_pos.y() , test_pos.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
        // ROS_INFO_COND(m_verbose," View found max");
        max_bool = true ; 
    }

    // ROS_INFO_COND(m_verbose," Angle should be between %f and %f", m_angle_low, m_angle_high);
    //We prioritize the value in the middle
    if(mid){
        findOrientation(mid_view) ; 
        isAngleok = verify_angle( frontier, mid_view , angle_mid, m_angle_low, m_angle_high ) ; 
        // ROS_INFO_COND(m_verbose," Angle found is %f", angle_mid);
        if( isAngleok ){
            // ROS_INFO_COND(m_verbose," Angle of mid candidate accepted  %f", angle_mid);
            // ROS_INFO_COND(m_verbose," mid view found %f %f %f", mid_view.x, mid_view.y, mid_view.z);

            Eigen::Vector3d mid_view_vec( mid_view.x, mid_view.y, mid_view.z);
            // ROS_INFO_COND(m_verbose," Mid view generated with distance of  %f", (mid_view_vec - frontier).norm() );
            
            vc= mid_view ; 
            return true;
        }
    }


    if(minimum){
        findOrientation(min_view) ; 
        isAngleok = verify_angle( frontier, min_view , angle_min, m_angle_low, m_angle_high ) ; 
        // ROS_INFO_COND(m_verbose," Angle found is  %f", angle_min);
        if( isAngleok ){
            // ROS_INFO_COND(m_verbose," Angle of min candidate accepted  %f", angle_min);
            // ROS_INFO_COND(m_verbose," min view found %f %f %f", min_view.x, min_view.y, min_view.z);

            Eigen::Vector3d min_view_vec( min_view.x, min_view.y, min_view.z);
            // ROS_INFO_COND(m_verbose," Min view generated with distance of  %f", (min_view_vec - frontier).norm() );

            vc = min_view ;
            return true;
        }
    }

    if (max_bool){
        findOrientation(max_view) ; 
        isAngleok = verify_angle( frontier, max_view , angle_max, m_angle_low, m_angle_high ) ; 
        // ROS_INFO_COND(m_verbose," Angle found is  %f", angle_max);
        if( isAngleok ){
            // ROS_INFO_COND(m_verbose," Angle of max candidate accepted  %f", angle_max);
            // ROS_INFO_COND(m_verbose," max view found %f %f %f", max_view.x, max_view.y, max_view.z);

            Eigen::Vector3d max_view_vec( max_view.x, max_view.y, max_view.z);
            // ROS_INFO_COND(m_verbose," Max view generated with distance of  %f", (max_view_vec - frontier).norm() );

            vc = max_view  ; 
            return true;

        }

    }

    // ROS_INFO_COND(m_verbose,"No position found : angle mid is  %f", angle_mid);
    // if( angle_mid > m_angle_high ){
    //     angle_diff = m_angle_high - angle_mid ;
    // }
    // else if( angle_mid < m_angle_low ){
    //     angle_diff = m_angle_low - angle_mid ;
    // }
    // else {
    //     angle_diff = angle_mid;

    // }

    if( view_angle.vert_angle_diff == 0 ){
        rejected_view = { test_pos.x() , test_pos.y() , test_pos.z() , 0,0,0,0, frontier.x() , frontier.y(), frontier.z()};
        rejected_view_candidates.push_back(rejected_view);
        ROS_INFO_COND(m_verbose," Angle correct but could not find any view ");

    }

    return false;




}

Eigen::Vector3d ViewGenerator::generateNoisePosition(const Eigen::Vector3d& current_pos, const Eigen::Vector3d& frontier, float distance, float noise_value ){
    Eigen::Vector3d new_pos = current_pos;


    //get the angle random noise rotation
    // float noise_value_radian = noise_value * M_PI / 180.0;
    // distribution_ = std::normal_distribution<double>(0.0, noise_value_radian);
    float gaussian_noise = distribution_(rng_);
    //ROS_INFO_COND(m_verbose," Gaussian noise is %f ", gaussian_noise);


    //translation before rotation at the origin
    new_pos.x() = new_pos.x()  - frontier.x()  ;
    new_pos.y() = new_pos.y() - frontier.y()  ;

    //rotation
    new_pos.x() = new_pos.x() * cos(gaussian_noise) - new_pos.y() * sin(gaussian_noise);
    new_pos.y() = new_pos.y() * cos(gaussian_noise) + new_pos.x() * sin(gaussian_noise);

    //translation back to the original
    new_pos.x() = new_pos.x() + frontier.x();
    new_pos.y() = new_pos.y() + frontier.y();

    //check the z component stays the original
    new_pos.z() = current_pos.z();

    //check the distance to frontiers is still the same
    // ROS_INFO_COND(m_verbose," Old distance is %f ", distance);
    float new_distance = ((current_pos - frontier).norm()) ; 
    // ROS_INFO_COND(m_verbose," New distance is %f ", new_distance);

    return new_pos;





}





// bool ViewGenerator::verify_angle2(const Eigen::Vector3d& gradient, const Eigen::Vector3d& vector){

//     //gradient and vector should already be normalized


//     //Compute signed vertical angle
//     //vertical_angle_rad = std::atan2(direction_original.z(), direction_original.head<2>().norm());
//     vertical_angle_rad = std::atan2(direction_original.z(), direction_proj.norm()); //same as last line
//     vertical_angle_deg = vertical_angle_rad * (180.0 / M_PI);

//     // ROS_INFO_COND(m_verbose,"Angle found is function is  %f", vertical_angle_deg);
//     angle = vertical_angle_deg;
//     if( (vertical_angle_deg < m_angle_high ) && (vertical_angle_deg > m_angle_low) ){
//         angle = vertical_angle_deg;
//         return true;
//     }

//     return false;



// }

bool ViewGenerator::isSafeView_Wrapper(const Eigen::Vector3d& voxel){
    if( m_forced_safety == true ){
        return isSafeView_Forced(voxel);
    }
    else{
        return isSafeView(voxel);
    }

}



bool ViewGenerator::isSafeView(const Eigen::Vector3d& voxel){
    float vox_size = m_map.getVoxelSize();

    if( isCorrectPos(voxel, m_bb) == false ){
        return false;
    }

    if ( isSafeGround(voxel) == false){
        return false;
    }

    char state ;    
    state = m_map.getVoxelState_ESDF(voxel) ;
    float distance = 0 ;
    // ROS_INFO_COND(m_verbose,"state of voxel (%f %f %f) in the ESDF is  '%c'", voxel.x(), voxel.y(), voxel.z(), state ) ;
    if( state == voxblox_map::VoxbloxMap::OCCUPIED ){
        return false;

    }

    // int min_radius = static_cast<int>(std::floor(-robot_radius_/2)) ; 
    // int max_radius = static_cast<int>(std::ceil(robot_radius_/2)) ;
    // for(int i= min_radius ; i <= max_radius ; i++){
    //     for(int j= min_radius ; j <= max_radius ; j++){
    //         for(int k= min_radius ; k <= max_radius ; k++){
    int steps = static_cast<int>(robot_radius_ / vox_size);  // 0.5 / 0.25 = 2


    for(int i = -steps; i <= steps; ++i) {
        for(int j = -steps; j <= steps; ++j) {
            for(int k = -steps; k <= steps; ++k) {

                Eigen::Vector3d shift = Eigen::Vector3d(i * vox_size ,j * vox_size ,k * vox_size );
                state = m_map.getVoxelState_ESDF(voxel + shift) ;
                distance = m_map.getVoxelDistance_ESDF(voxel + shift) ;
                //state = m_map.getVoxelState_TSDF(voxel + shift, 0);
                // ROS_INFO_COND(m_verbose,"state of voxel (%f %f %f) in the ESDF is  '%d'", (voxel+shift).x(), (voxel+shift).y(), (voxel+shift).z(),  static_cast<int>(state) ) ;
                //if ( distance <= 1 ){
                if( state == voxblox_map::VoxbloxMap::OCCUPIED ){
                    //ROS_INFO_COND(m_verbose,"FALSE");
                    return false;
                }

            }
        }
    }
    // ROS_INFO_COND(m_verbose,"TRUE");
    return true;

}

bool ViewGenerator::isSafeView_Forced(const Eigen::Vector3d& voxel){
    float vox_size = m_map.getVoxelSize();

    if( isCorrectPos(voxel, m_bb) == false ){
        return false;
    }

    if ( isSafeGround(voxel) == false){
        return false;
    }

    char state ;    
    state = m_map.getVoxelState_ESDF(voxel) ;
    float distance = 0 ;
    ROS_INFO_COND(m_verbose,"state of voxel (%f %f %f) in the ESDF is  '%s'", voxel.x(), voxel.y(), voxel.z(), voxelStateToString(state) ) ;
    if( state == voxblox_map::VoxbloxMap::OCCUPIED || state == voxblox_map::VoxbloxMap::UNKNOWN ){
        return false;

    }

    // if( state == voxblox_map::VoxbloxMap::FREE &&  m_map.getVoxelDistance_ESDF(voxel) < robot_radius){
    //     return false;
    // }

    // int min_radius = static_cast<int>(std::floor(-robot_radius_/2)) ; 
    // int max_radius = static_cast<int>(std::ceil(robot_radius_/2)) ;

    // float min_radius = -robot_radius_ ; 
    // float max_radius = robot_radius_ ;

    int steps = static_cast<int>(robot_radius_ / vox_size);  // 0.5 / 0.25 = 2
    
    int count_voxels_ok = 0 ; 
    int count_voxels_fail = 0;
    bool isViewSafe = true;

    for(int i = -steps; i <= steps; ++i) {
        for(int j = -steps; j <= steps; ++j) {
            for(int k = -steps; k <= steps; ++k) {

                Eigen::Vector3d shift = Eigen::Vector3d(i * vox_size ,j * vox_size ,k * vox_size );
                //Eigen::Vector3d shift = Eigen::Vector3d(i  ,j  ,k );
                //ROS_INFO_COND(m_verbose," Shift voxel (%f %f %f)  ", shift.x(), shift.y(), shift.z() ) ;
                state = m_map.getVoxelState_ESDF(voxel + shift) ;
                distance = m_map.getVoxelDistance_ESDF(voxel + shift) ;
                //state = m_map.getVoxelState_TSDF(voxel + shift, 0);
                //ROS_INFO_COND(m_verbose," Radius state of voxel (%f %f %f) in the ESDF is  '%d'", (voxel+shift).x(), (voxel+shift).y(), (voxel+shift).z(),  static_cast<int>(state) ) ;
                //if ( distance <= 1 ){
                ROS_INFO_COND(m_verbose,"state of voxel is  '%s'", voxelStateToString(state) ) ;
                if( state == voxblox_map::VoxbloxMap::OCCUPIED || state == voxblox_map::VoxbloxMap::UNKNOWN ){
                    //ROS_INFO_COND(m_verbose,"FALSE");
                    //ROS_INFO_COND(m_verbose,"Tried %d voxels before failing safety");
                    isViewSafe = false;
                    count_voxels_fail = count_voxels_fail +1 ;
                    //return false;
                }
                else{
                    count_voxels_ok = count_voxels_ok +1 ;
                }

            }
        }
    }
    // ROS_INFO_COND(m_verbose,"TRUE");
    ROS_INFO_COND(m_verbose, "voxel safety :  %s", isViewSafe? "true" : "false");
    ROS_INFO_COND(m_verbose, " Safe voxels are %f", (count_voxels_ok/(count_voxels_ok + count_voxels_fail)) );
    ROS_INFO_COND(m_verbose, " Evaluated voxels are %d", (count_voxels_ok + count_voxels_fail) );
    //return true;
    return isViewSafe ; 
}




// bool ViewGenerator::isSafeView(const Eigen::Vector3d& voxel){
//     //isFree = (current_state == voxblox_map::VoxbloxMap::FREE);
//     //const voxblox::EsdfVoxel& voxel = 
//     float dist = m_map.getVoxelDistance_ESDF(voxel) ;
//     if( dist < (robot_radius_ * m_map.getVoxelSize() * 0.5 ) ){
//         return false;
//     }
//     for(int i= -std::ceil(robot_radius_/2) ; i < std::ceil(robot_radius_/2) ; i++){
//         for(int j= -std::ceil(robot_radius_/2) ; i < std::ceil(robot_radius_/2) ; i++){
//             for(int k= -std::ceil(robot_radius_/2) ; i < std::ceil(robot_radius_/2) ; k++){

//                 Eigen::Vector3d shift = Eigen::Vector3d(i,j,k);
//                 dist = m_map.getVoxelDistance_ESDF(voxel + shift) ;
//                 if( dist < m_map.getVoxelSize() ){
//                     return false;
//                 }

//             }
//         }
//     }
//     return true;

// }



Eigen::Vector3d ViewGenerator::computeGradient(const Eigen::Vector3d& voxel){
    

    float vs = m_map.getVoxelSize() ; 
    Eigen::Vector3d gradient_vector(0,0,0); 
    Eigen::Vector3d shift_x(vs,0,0) ; 
    Eigen::Vector3d shift_y(0,vs,0) ; 
    Eigen::Vector3d shift_z(0,0,vs) ; 

    // gradient_vector.x() = m_map.getVoxelDistance_ESDF( voxel + shift_x) - m_map.getVoxelDistance_ESDF( voxel - shift_x);
    // gradient_vector.y() = m_map.getVoxelDistance_ESDF( voxel + shift_y) - m_map.getVoxelDistance_ESDF( voxel - shift_y);
    // gradient_vector.z() = m_map.getVoxelDistance_ESDF( voxel + shift_z) - m_map.getVoxelDistance_ESDF( voxel - shift_z);

    // ROS_INFO_COND(m_verbose,"List voxels analyzed");

    // Eigen::Vector3d center;
    // if (m_map.getVoxelCenter_ESDF(&center, voxel)) {
    //     ROS_INFO_COND(m_verbose,"%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel + shift_x)) {
    //     ROS_INFO_COND(m_verbose,"%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel - shift_x)) {
    //     ROS_INFO_COND(m_verbose,"%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel + shift_y)) {
    //     ROS_INFO_COND(m_verbose,"%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel - shift_y)) {
    //     ROS_INFO_COND(m_verbose,"%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel + shift_z)) {
    //     ROS_INFO_COND(m_verbose,"%f %f %f", center.x(), center.y(), center.z());
    // }
    // if (m_map.getVoxelCenter_ESDF(&center, voxel - shift_z)) {
    //     ROS_INFO_COND(m_verbose,"%f %f %f", center.x(), center.y(), center.z());
    // }
    
    // ROS_INFO_COND(m_verbose,"End list");


    
    

    gradient_vector.x() = m_map.getVoxelDistance_ESDF( voxel + shift_x) - m_map.getVoxelDistance_ESDF( voxel - shift_x);
    gradient_vector.y() = m_map.getVoxelDistance_ESDF( voxel + shift_y) - m_map.getVoxelDistance_ESDF( voxel - shift_y);
    gradient_vector.z() = m_map.getVoxelDistance_ESDF( voxel + shift_z) - m_map.getVoxelDistance_ESDF( voxel - shift_z);

    gradient_vector.x() = gradient_vector.x() / (2 *vs) ; 
    gradient_vector.y() = gradient_vector.y() / (2*vs) ;
    gradient_vector.z() = gradient_vector.z() / (2*vs);

    if (gradient_vector.norm() == 0){
        //if gradient = 0 use forward difference
        // ROS_INFO_COND(m_verbose,"Forward difference");


        gradient_vector.x() = ( m_map.getVoxelDistance_ESDF( voxel + shift_x) - m_map.getVoxelDistance_ESDF( voxel ) ) / vs ;
        gradient_vector.y() = ( m_map.getVoxelDistance_ESDF( voxel + shift_y) - m_map.getVoxelDistance_ESDF( voxel ) ) / vs ;
        gradient_vector.z() = (m_map.getVoxelDistance_ESDF( voxel + shift_z) - m_map.getVoxelDistance_ESDF( voxel ) ) /vs ;

    }

    if (gradient_vector.norm() == 0){
        // ROS_INFO_COND(m_verbose,"Gradient hardouin");

        //if gradient = 0 use hardouin weighted based method 

        return computeGradient_26(voxel); 

    }
    
    


    //Correction

    // if( gradient_vector.x() > 0){
    //     gradient_vector.x() = std::ceil( gradient_vector.x() ) ;
    // }
    // else{
    //     gradient_vector.x() = std::floor( gradient_vector.x() ) ;
    // }

    // if( gradient_vector.y() > 0){
    //     gradient_vector.y() = std::ceil( gradient_vector.y() ) ;
    // }
    // else{
    //     gradient_vector.y() = std::floor( gradient_vector.y() ) ;
    // }

    // if( gradient_vector.z() > 0){
    //     gradient_vector.z() = std::ceil( gradient_vector.z() ) ;
    // }
    // else{
    //     gradient_vector.z() = std::floor( gradient_vector.z() ) ;
    // }

    return gradient_vector;

}


//Hardouin method 
Eigen::Vector3d ViewGenerator::computeGradient_26(const Eigen::Vector3d& voxel){
    // ROS_INFO_COND(m_verbose,"Gradient hardouin");


    Eigen::Vector3d grad_dir ; 
    Eigen::Vector3d temp_dir ; 
    double weight = 0 ; 
    for(int i = 0; i< 26;i++){
        
        weight = m_map.getVoxelWeight_TSDF( voxel + c_neighbor_voxels_[i] ) ; 
        if( weight > 0 && m_map.getVoxelDistance_TSDF( voxel + c_neighbor_voxels_[i] ) < m_map.getVoxelSize() ){
            weight = -weight ; 
        }
        temp_dir = c_neighbor_voxels_[i].normalized() ; 
        grad_dir = grad_dir + (weight * temp_dir) ; 
        

    }

    grad_dir.normalize();

    return grad_dir ; 


}

Eigen::Vector3d ViewGenerator::computeGradient_Sobel(const Eigen::Vector3d& voxel){
    Eigen::Vector3d vect;
    return vect;
}



// Eigen::Vector3d ViewGenerator::computeGradient2(const Eigen::Vector3d& voxel){
    

//     float vs = 1 ; 
//     Eigen::Vector3d gradient_vector(0,0,0); 
//     Eigen::Vector3d shift_x(vs,0,0) ; 
//     Eigen::Vector3d shift_y(0,vs,0) ; 
//     Eigen::Vector3d shift_z(0,0,vs) ; 
    



// }


// ViewGenerator::ViewGenerator(/* args */)
// {
// }

// ViewGenerator::~ViewGenerator()
// {
// }
