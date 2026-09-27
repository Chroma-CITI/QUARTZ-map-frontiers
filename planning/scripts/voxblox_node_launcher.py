#!/usr/bin/env python3
# coding: utf-8
import os
import rospy
from std_srvs.srv import Empty

class NodeLauncher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('node_launcher', anonymous=True)
        rospy.loginfo("[node_launcher] Starting node ...")
        self._rate = rospy.Rate(0.1)
        self._is_manual_launch = os.environ.get('MANUAL_LAUNCH', 'true')
        rospy.loginfo("manual launch: %s", self._is_manual_launch)

        # Get parameters
        # Expecting topics as follow. /droneName_NUMBER/serviceName
        # FIXME: this architecture forces the param /drone_name to be already loaded to the rosparams
        drone_name = rospy.get_param('/drone_name')
        self._topic_namespace = "/" + drone_name + "_"
        self._services_name = rospy.get_param('~service_name', ["start_NBV_selector", "mav_local_planner/activate_node"])
        self._number_of_drones = rospy.get_param('/number_of_drones')

        # Make sure gazebo service is running before trying to start simulation
        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.sleep(2)

        # Service server
        self._launch_service = rospy.Service('/start_voxblox_nodes', Empty, self._LaunchVoxbloxNodeCallback)

        # Service clients
        self._service_clients = list()

        # Add voxblox to the service client list
        voxblox_service_name = "/voxblox_node/activate_node"
        rospy.wait_for_service(voxblox_service_name)
        self._service_clients.append(rospy.ServiceProxy(voxblox_service_name, Empty))
        rospy.loginfo("[node_launcher] Added service /voxblox_node/activate_node to the list of services to call")

        # Should work for N number of service
        for drone_id in range(self._number_of_drones):
            for service_name in self._services_name:
                full_service_name = self._topic_namespace + str(drone_id) + '/' + service_name
                # Wait for the service to be available
                rospy.wait_for_service(full_service_name)
                rospy.loginfo("[node_launcher] Added service %s to the list of services to call", full_service_name)
                self._service_clients.append(rospy.ServiceProxy(full_service_name, Empty))

    def _LaunchVoxbloxNodeCallback(self, req):
        self._CallServiceClients()
        return []
    
    def _CallServiceClients(self):
        rospy.loginfo("Calling service clients...")
        for client in self._service_clients:
            try:
                client() # Call the service
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

    def Spin(self):
        while not rospy.is_shutdown():
            if self._is_manual_launch == "false":
                self._CallServiceClients()
            self._rate.sleep()

if __name__ == '__main__':
    try:
        node_laucher = NodeLauncher()
        node_laucher.Spin()
    except rospy.ROSInterruptException:
        pass
