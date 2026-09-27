#pragma once

#include <voxblox_map/utils.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "planning/data/type_conversions.h"

namespace map_frontiers { namespace visualization {

visualization_msgs::Marker CreateHorizontalFOVMarker(const geometry_msgs::Pose& view, int markerId, float range) {
    // Define the Marker message
    visualization_msgs::Marker fovMarker;
    fovMarker.header.frame_id = "world";  // Reference frame
    fovMarker.header.stamp = ros::Time::now();
    fovMarker.ns = "horizontal_fov";
    fovMarker.id = markerId;
    fovMarker.type = visualization_msgs::Marker::CYLINDER;
    fovMarker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker (position and orientation) and its scales
    fovMarker.pose = view;
    fovMarker.scale.x = range;  // radius of the cylinder
    fovMarker.scale.y = range;  // radius of the cylinder
    fovMarker.scale.z = 0.05;  // height of the cylinder

    // Set the color (RGBA)
    fovMarker.color.r = 1.0f;
    fovMarker.color.g = 0.0f;
    fovMarker.color.b = 0.0f;
    fovMarker.color.a = 0.6f;

    return fovMarker;
    }

visualization_msgs::Marker CreateVerticalFOVMarker(const geometry_msgs::Pose& view, int markerId, float range) {
    // Define the Marker message
    visualization_msgs::Marker fovMarker;
    fovMarker.header.frame_id = "world";  // Reference frame
    fovMarker.header.stamp = ros::Time::now();
    fovMarker.ns = "vertical_fov";
    fovMarker.id = markerId;
    fovMarker.type = visualization_msgs::Marker::LINE_STRIP;  // Use LINE_STRIP to draw lines
    fovMarker.action = visualization_msgs::Marker::ADD;

    // Set the color and scale of the triangle
    fovMarker.color.r = 1.0;
    fovMarker.color.g = 0.0;
    fovMarker.color.b = 0.0;
    fovMarker.color.a = 0.5;  // Semi-transparent red

    fovMarker.scale.x = 0.05;  // Thickness of the lines

    // Set the range and FOV angle
    float angleFovUp = 57.0 * M_PI / 180.0;  // Half of the vertical FOV, in rad
    float angleFovDown = 25.0 * M_PI / 180.0;  // Half of the vertical FOV, in rad

    // Define the points for the vertical fov triangle in the local drone frame
    float topOffsetInZ = range / sin(angleFovDown);
    float downOffsetInZ = range / sin(angleFovUp);
    tf2::Vector3 topPointLocal(range, 0.0, angleFovUp);
    tf2::Vector3 downPointLocal(range, 0.0, -angleFovDown);
    
    // Project them in world frame
    tf2::Transform worldToDroneTransform;
    conversions::GeometryPoseToTf2Transform(view, worldToDroneTransform);
    tf2::Vector3 topPointGlobal = worldToDroneTransform * topPointLocal;
    tf2::Vector3 downPointGlobal = worldToDroneTransform * downPointLocal; 

    // Add them to the marker's point list
    geometry_msgs::Point topGeometryPointGlobal, downGeometryPointGlobal;
    conversions::tf2VectorToGeometryPoint(topPointGlobal, topGeometryPointGlobal);
    conversions::tf2VectorToGeometryPoint(downPointGlobal, downGeometryPointGlobal);

    fovMarker.points.push_back(view.position);
    fovMarker.points.push_back(topGeometryPointGlobal);
    fovMarker.points.push_back(downGeometryPointGlobal);
    fovMarker.points.push_back(view.position);
    return fovMarker;
}

visualization_msgs::Marker CreateViewToFrontiersLine(const geometry_msgs::Pose& view, const geometry_msgs::Point& frontier, int markerId) {
    // Define the Marker message
    visualization_msgs::Marker fovMarker;
    fovMarker.header.frame_id = "world";  // Reference frame
    fovMarker.header.stamp = ros::Time::now();
    fovMarker.ns = "view_to_frontiers";
    fovMarker.id = markerId;
    fovMarker.type = visualization_msgs::Marker::LINE_STRIP;  // Use LINE_STRIP to draw lines
    fovMarker.action = visualization_msgs::Marker::ADD;

    // Set the color and scale of the triangle
    fovMarker.color.r = 1.0;
    fovMarker.color.g = 0.0;
    fovMarker.color.b = 0.0;
    fovMarker.color.a = 0.5;  // Semi-transparent red

    fovMarker.scale.x = 0.01;  // Thickness of the lines

    fovMarker.points.push_back(view.position);
    fovMarker.points.push_back(frontier);
    return fovMarker;
}

visualization_msgs::Marker CreateBoundingBoxText(const BoundingBox& bounding_box, float quality, std::string box_ns="bounding_box", int id=0) {
    float x_center = (bounding_box.x_max + bounding_box.x_min) / 2.0;
    float y_center = (bounding_box.y_max + bounding_box.y_min) / 2.0;
    float z_center = (bounding_box.z_max + bounding_box.z_min) / 2.0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // Reference frame
    marker.header.stamp = ros::Time::now();
    marker.ns = box_ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_center;
    marker.pose.position.y = y_center;
    marker.pose.position.z = z_center;
    marker.pose.orientation.w = 1.0;  // neutral orientation
    marker.scale.z = 0.2;  // text size
    marker.color.a = 1.0;  // alpha (opacity)
    marker.color.r = 1.0;  // red
    marker.color.g = 1.0;  // green
    marker.color.b = 1.0;  // blue
    marker.text = "Bounding box of quality " + std::to_string(quality);

    marker.lifetime = ros::Duration();  // forever

    return marker;

}

visualization_msgs::Marker CreateBoundingBoxVisualization(const BoundingBox& bounding_box, std::string box_ns="bounding_box", int id=0) {
    // Compute size and center of the box
    float x_size = bounding_box.x_max - bounding_box.x_min;
    float y_size = bounding_box.y_max - bounding_box.y_min;
    float z_size = bounding_box.z_max - bounding_box.z_min;

    float x_center = (bounding_box.x_max + bounding_box.x_min) / 2.0;
    float y_center = (bounding_box.y_max + bounding_box.y_min) / 2.0;
    float z_center = (bounding_box.z_max + bounding_box.z_min) / 2.0;

    // Fill up the visualization marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // Reference frame
    marker.header.stamp = ros::Time::now();
    marker.ns = box_ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose (center of box)
    marker.pose.position.x = x_center;
    marker.pose.position.y = y_center;
    marker.pose.position.z = z_center;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set scale (size of box)
    marker.scale.x = x_size;
    marker.scale.y = y_size;
    marker.scale.z = z_size;

    // Set color (RGBA)
    if (box_ns == "bounding_box") {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.3f;  // semi-transparent
    } else {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.3f;  // semi-transparent
    }

    marker.lifetime = ros::Duration();  // forever

    return marker;
}


} //namespace visualization

} //namespace map_frontiers
