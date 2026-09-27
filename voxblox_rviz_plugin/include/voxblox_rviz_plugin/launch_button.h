#pragma once

#include <rviz/panel.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

namespace voxblox_rviz_plugin {

class LaunchButton : public rviz::Panel
{
  Q_OBJECT

public:
  LaunchButton(QWidget* parent = 0);
  virtual ~LaunchButton();

public Q_SLOTS:
  void onButtonPressed();

private:
  QPushButton* button_;
  ros::NodeHandle nh_;
  ros::ServiceClient service_client_;
};

}  // end namespace voxblox_rviz_plugin
