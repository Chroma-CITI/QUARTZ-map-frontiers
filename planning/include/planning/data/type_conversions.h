#pragma once

// #include <Eigen/Eigen>
#include "planning/modules/ViewGenerator.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace map_frontiers { namespace conversions{

/*

    Conversion of ViewCandidate to another type

*/

/// @brief Convert a ViewCandidate into a geometry_msgs::Pose.
/// @param viewIn  the ViewCandidate to be converted
/// @param poseOut the resulting geometry_msgs::Pose
void ViewCandidateToGeometryPose(const ViewCandidate& viewIn, geometry_msgs::Pose& poseOut) {
    poseOut.position.x = viewIn.x; 
    poseOut.position.y = viewIn.y;
    poseOut.position.z = viewIn.z; 

    poseOut.orientation.x = viewIn.q_x; 
    poseOut.orientation.y = viewIn.q_y; 
    poseOut.orientation.z = viewIn.q_z; 
    poseOut.orientation.w = viewIn.q_w;
}

void ViewCandidateTf2VectorAndQuaternion(const ViewCandidate& viewIn, tf2::Vector3& positionOut, tf2::Quaternion& orientationOut) {
    positionOut = tf2::Vector3(viewIn.x, viewIn.y, viewIn.z);
    orientationOut = tf2::Quaternion(viewIn.q_w, viewIn.q_x, viewIn.q_y, viewIn.q_z);
}

void ViewCandidateToTf2Transform(const ViewCandidate& viewIn, tf2::Transform& transformOut) {
    tf2::Vector3 translation;
    tf2::Quaternion rotation;
    ViewCandidateTf2VectorAndQuaternion(viewIn, translation, rotation);
    transformOut.setOrigin(translation);
    transformOut.setRotation(rotation);
}

/// @brief Convert a ViewCandidate into a Eigen pose and orientation.
/// Note: it is note the same order as a geometry_msg quaternion!
/// ViewCandidate and quaternion is qw, qx, qy qz where geometry_msg::Quaternion is qx, qy, qz, qw
/// @param viewIn the ViewCandidate to be converted
/// @param position the resulting Eigen::Vector3d position
/// @param orientation the resulting Eigen::Quaterniond orientation
// void ViewCandidateToGeometryPose(const ViewCandidate& viewIn, Eigen::Vector3d& positionOut, Eigen::Quaterniond& orientationOut) {
//     positionOut = Eigen::Vector3d( viewIn.x, viewIn.y, viewIn.z);
//     orientationOut = Eigen::Quaterniond(viewIn.q_w, viewIn.q_x, viewIn.q_y, viewIn.q_z);
// }

/*

    Conversion of geometry_msgs::Pose to another type

*/

void GeometryPoseToTf2Transform(const geometry_msgs::Pose& poseIn, tf2::Transform& transformOut) {
    tf2::Quaternion rotation;
    tf2::fromMsg(poseIn.orientation, rotation);  // Convert quaternion to tf2::Quaternion
    tf2::Vector3 translation(poseIn.position.x, poseIn.position.y, poseIn.position.z);  // Position
    transformOut.setOrigin(translation);
    transformOut.setRotation(rotation);
}

/*

    Conversion of tf2 struct to another type

*/

void tf2VectorToGeometryPoint(const tf2::Vector3& positionIn, geometry_msgs::Point& pointOut) {
    pointOut.x = positionIn.x();
    pointOut.y = positionIn.y();
    pointOut.z = positionIn.z();
}

} //namespace conversions

} //namespace map_frontiers
