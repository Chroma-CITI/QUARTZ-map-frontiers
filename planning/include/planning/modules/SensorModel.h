#pragma once
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <voxblox_map/voxblox_map.h>
#include <string>
#include <cmath>

class SensorModel
{
private:
    

    

public:

    // mounting transform from body (pose) to sensor , in body frame
    Eigen::Vector3d mounting_translation_;  // x,y,z [m]
    Eigen::Quaterniond mounting_rotation_;  // x,y,z,w quaternion

    //sensor parameters
    double p_ray_length_;  // params for camera model
    double p_fov_x_;  // Total fields of view [deg], expected symmetric w.r.t.
    // sensor facing direction
    double p_fov_y_;
    int p_resolution_x_;
    int p_resolution_y_;
    double p_sampling_time_;  // sample camera poses from segment, use 0 for last
                                // only
    SensorModel(double ray_length, double fov_x, double fov_y, int resolution_x, int resolution_y, double sampling_time);
    SensorModel();
    ~SensorModel();
    void getDirectionVector_LIDAR(Eigen::Vector3d* result, double relative_x,
                                    double relative_y);
};

SensorModel::SensorModel(double ray_length, double fov_x, double fov_y, int resolution_x, int resolution_y, double sampling_time) : p_ray_length_{ray_length}, p_fov_x_{fov_x}, p_fov_y_{fov_y}, p_resolution_x_{resolution_x}, p_resolution_y_{resolution_y}, p_sampling_time_{sampling_time}
{
}

SensorModel::SensorModel(){}

SensorModel::~SensorModel()
{
}

void SensorModel::getDirectionVector_LIDAR(Eigen::Vector3d* result, double relative_x,
                                    double relative_y) {
  double phi = (0.5 - relative_x) * p_fov_x_;
  double theta = M_PI / 2.0 + (relative_y - 0.5) * p_fov_y_;
  *result =
      Eigen::Vector3d(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}