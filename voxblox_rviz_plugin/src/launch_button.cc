#include "voxblox_rviz_plugin/launch_button.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>

namespace voxblox_rviz_plugin {

LaunchButton::LaunchButton(QWidget* parent)
  : rviz::Panel(parent)
{
  // Create a button
  button_ = new QPushButton("Launch voxblox nodes", this);

  // Create a layout and add the button to it
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(button_);
  setLayout(layout);

  // Connect the button click to the service call
  connect(button_, SIGNAL(clicked()), this, SLOT(onButtonPressed()));

  // Initialize the ROS service client
  service_client_ = nh_.serviceClient<std_srvs::Empty>("/start_voxblox_nodes");
}

LaunchButton::~LaunchButton()
{
}

void LaunchButton::onButtonPressed()
{
  // Call the ROS service when the button is pressed
  std_srvs::Empty srv;
  if (service_client_.call(srv))
  {
    ROS_INFO("Service called successfully");
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }
}

}  // end namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::LaunchButton, rviz::Panel)
