#!/usr/bin/env python
"""Controls the UR5 arm (Ur5_2) near the bins to pick up boxes from the conveyor. """
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from hrwros_gazebo.msg import LogicalCameraImage
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import tf2_ros
import math
import time
import sys
import copy
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest
from std_srvs.srv import Empty
from pkg_ros_iot_bridge.msg import *  # Message Class that is used for Result Messages


class Camera1:
    """Class for Camera1"""
    def __init__(self, arg_robot_name):
        """Initialize the Constructor"""

        self._robot_ns = '/' + arg_robot_name
        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns + "/robot_description",
                                                      ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
                                                          robot_description=self._robot_ns + "/robot_description",
                                                          ns=self._robot_ns)
        self._group.set_planning_time(5)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns + '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns + '/execute_trajectory',
                                                                        moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''
        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_moveit_examples')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
        self.enum = []
        self.color = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        self.logicalcam2_sub = rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, self.sub_callback)
        self._pospkg = 0
        self._pospkg_x = 0
        self._pospkg_y = 0
        self._pospkg_z = 0
        self._pub2 = rospy.Publisher("Talker", String, queue_size=10)
        self.sub = rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, self.sub_callbackmsg)
        self.received_msg = []
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def clear_octomap(self):
        """Clear Octomap"""

        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        """End Effector Cartesian Translation
            parameters:
            trans_x (float): Translation in x
            trans_y (float): Translation in y
            trans_y (float): Translation in z
            orienx (float): orientation in x
            orieny (float): orientation in y
            orienz (float): orientation in z
            orienw (float): orientation in w
                """
        # 1. Create a empty list to hold waypoints
        waypoints = []
        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)
        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5
        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))
        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)  # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]
        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def set_joint_angles(self, arg_list_joint_angles):
        """Planning to a Joint Angle Goal"""
        list_joint_values = self._group.get_current_joint_values()

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        pose_values = self._group.get_current_pose().pose
        if (flag_plan == True):
            pass
        else:
            pass
        return flag_plan

    def go_to_pose(self, arg_pose):
        """Planning to a Go to pose Goal"""

        pose_values = self._group.get_current_pose().pose
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        pose_values = self._group.get_current_pose().pose
        list_joint_values = self._group.get_current_joint_values()
        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        """Destructor"""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

    def get_qr_data(self, arg_image):
        """
                Get the data From image and process Data.
            """
        global e
        qr_result = decode(arg_image)
        # initializing list
        print (len(qr_result))
        for i in range(0, len(qr_result)):
            a = qr_result[i].data
            b = qr_result[i].rect[0]
            c = qr_result[i].rect[1]
            e = {"left": b, "top": c, "color": a}
            self.enum = self.enum + [e]

        self.enum.sort(key=lambda x: x.get('left'))
        print (self.enum)

        for i in range(0, len(qr_result)):
            if 313 < self.enum[i]["top"] < 320 and 126 < self.enum[i]["left"] < 133:
                self.color[0] = self.enum[i]["color"]

            if 313 < self.enum[i]["top"] < 320 and 312 < self.enum[i]["left"] < 320:
                self.color[1] = self.enum[i]["color"]

            if 313 < self.enum[i]["top"] < 320 and 500 < self.enum[i]["left"] < 508:
                self.color[2] = self.enum[i]["color"]

            if 492 < self.enum[i]["top"] < 500 and 126 < self.enum[i]["left"] < 133:
                self.color[3] = self.enum[i]["color"]

            if 492 < self.enum[i]["top"] < 500 and 312 < self.enum[i]["left"] < 320:
                self.color[4] = self.enum[i]["color"]

            if 492 < self.enum[i]["top"] < 500 and 500 < self.enum[i]["left"] < 508:
                self.color[5] = self.enum[i]["color"]

            if 640 < self.enum[i]["top"] < 648 and 126 < self.enum[i]["left"] < 133:
                self.color[6] = self.enum[i]["color"]

            if 640 < self.enum[i]["top"] < 648 and 312 < self.enum[i]["left"] < 320:
                self.color[7] = self.enum[i]["color"]

            if 640 < self.enum[i]["top"] < 648 and 500 < self.enum[i]["left"] < 508:
                self.color[8] = self.enum[i]["color"]

            if 792 < self.enum[i]["top"] < 799 and 126 < self.enum[i]["left"] < 133:
                self.color[9] = self.enum[i]["color"]

            if 792 < self.enum[i]["top"] < 799 and 312 < self.enum[i]["left"] < 320:
                self.color[10] = self.enum[i]["color"]

            if 792 < self.enum[i]["top"] < 799 and 500 < self.enum[i]["left"] < 508:
                self.color[11] = self.enum[i]["color"]

        print (self.color)

    def callback(self, data):
        """This is a callback function of subscriber /eyrc/vb/camera_1/image_raw """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_sub.unregister()

        except CvBridgeError as e:
            rospy.logerr(e)
        (rows, cols, channels) = cv_image.shape
        #result = cv2.fastNlMeansDenoisingColored(cv_image, None, 20, 10, 7, 21)
        image = cv_image
        # Resize a 720x1280 image to 360x640 to fit it on the screen
        """resized_image = cv2.resize(image, (720 / 2, 1280 / 2))
        cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
        rospy.loginfo(self.get_qr_data(image))"""
        _,threshold = cv2.threshold(image, 70, 255, cv2.THRESH_TRUNC)
        self.get_qr_data(threshold)
        cv2.waitKey(3)

    def sub_callback(self, msg):
        """An Callback Function from locical Camera2"""
        listpkg = msg.models
        for i in listpkg:
            self._pospkg = i.type

    def sub_callbackmsg(self, msg):
        """This is a callback function of  MQTT Subscriptions /ros_iot_bridge/mqtt/sub"""

        print (msg.message)
        self.received_msg = self.received_msg + [msg.message]
        print (self.received_msg)

    def func_tf_print(self, arg_frame_1, arg_frame_2):
        """ This is a function which print object postion with respect to tf"""
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
            self._pospkg_x = trans.transform.translation.x
            self._pospkg_y = trans.transform.translation.y
            self._pospkg_z = trans.transform.translation.z


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")


def main():
    """Main"""
    rospy.init_node('task6node2', anonymous=True)
    rospy.sleep(10)
    rospy.sleep(10)
    ur5 = Camera1(sys.argv[1])
    rospy.sleep(10)
    rospy.sleep(10)
    rospy.sleep(10)
    rospy.sleep(10)



    rospy.wait_for_service("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2")
    client2 = rospy.ServiceProxy("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2", vacuumGripper)

    box_length = 0.15  # Length of the Package
    vacuum_gripper_width = 0.115  # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length / 2)  # 0.19
    # Teams may use this info in Tasks

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length / 2)
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    ur5_2_home_posered = geometry_msgs.msg.Pose()
    ur5_2_home_posered.position.x = 0.179
    ur5_2_home_posered.position.y = 0.805
    ur5_2_home_posered.position.z = 1.2
    ur5_2_home_posered.orientation.x = -0.5
    ur5_2_home_posered.orientation.y = -0.5
    ur5_2_home_posered.orientation.z = 0.5
    ur5_2_home_posered.orientation.w = 0.5

    ur5_2_home_poseyellow = geometry_msgs.msg.Pose()
    ur5_2_home_poseyellow.position.x = 0.815
    ur5_2_home_poseyellow.position.y = 0.215
    ur5_2_home_poseyellow.position.z = 1.2
    ur5_2_home_poseyellow.orientation.x = -0.5
    ur5_2_home_poseyellow.orientation.y = -0.5
    ur5_2_home_poseyellow.orientation.z = 0.5
    ur5_2_home_poseyellow.orientation.w = 0.5

    ur5_2_home_posegreen = geometry_msgs.msg.Pose()
    ur5_2_home_posegreen.position.x = 0.112
    ur5_2_home_posegreen.position.y = -0.817
    ur5_2_home_posegreen.position.z = 1.2
    ur5_2_home_posegreen.orientation.x = -0.5
    ur5_2_home_posegreen.orientation.y = -0.5
    ur5_2_home_posegreen.orientation.z = 0.5
    ur5_2_home_posegreen.orientation.w = 0.5

    while not rospy.is_shutdown():
        # ur5.go_to_pose(ur5_2_home_pose_1)
        ur5.go_to_pose(ur5_2_home_pose)
        rospy.wait_for_service("/eyrc/vb/conveyor/set_power")
        clientbelt = rospy.ServiceProxy("/eyrc/vb/conveyor/set_power", conveyorBeltPowerMsg)
        requestbelt = conveyorBeltPowerMsgRequest(100)
        clientbelt(requestbelt)
        for i in range(len(ur5.received_msg)):
            while ur5._pospkg == "packagen00":
                target_frame = "logical_camera_2_packagen00_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[0] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))


                    elif ((ur5.color[0] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))
                    elif ((ur5.color[0] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break

            while ur5._pospkg == "packagen01":
                target_frame = "logical_camera_2_packagen01_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[1] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))



                    elif ((ur5.color[1] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))


                    elif ((ur5.color[1] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)

                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break
            while ur5._pospkg == "packagen02":
                target_frame = "logical_camera_2_packagen02_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:

                    if ((ur5.color[2] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[2] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[2] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break

            while ur5._pospkg == "packagen10":
                target_frame = "logical_camera_2_packagen10_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[3] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[3] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[3] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break
            while ur5._pospkg == "packagen11":
                target_frame = "logical_camera_2_packagen11_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[4] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[4] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[4] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break

            while ur5._pospkg == "packagen12":
                target_frame = "logical_camera_2_packagen12_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[5] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[5] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[5] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break

            while ur5._pospkg == "packagen20":
                target_frame = "logical_camera_2_packagen20_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[6] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[6] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[6] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break
            while ur5._pospkg == "packagen21":
                target_frame = "logical_camera_2_packagen21_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[7] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[7] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[7] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break
            while ur5._pospkg == "packagen22":
                target_frame = "logical_camera_2_packagen22_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[8] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[8] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[8] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break
            while ur5._pospkg == "packagen30":
                target_frame = "logical_camera_2_packagen30_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[9] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[9] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[9] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break
            while ur5._pospkg == "packagen31":
                target_frame = "logical_camera_2_packagen31_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[10] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[10] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[10] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break
            while ur5._pospkg == "packagen32":
                target_frame = "logical_camera_2_packagen32_frame"
                reference_frame = "world"
                ur5.func_tf_print(reference_frame, target_frame)

                if -0.1 < ur5._pospkg_y < 0.1:
                    if ((ur5.color[11] == "red") and (ur5.received_msg[i] == "HP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posered)
                        ur5.go_to_pose(ur5_2_home_posered)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[11] == "green") and (ur5.received_msg[i] == "LP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        ur5.go_to_pose(ur5_2_home_posegreen)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                    elif ((ur5.color[11] == "yellow") and (ur5.received_msg[i] == "MP")):
                        requestbelt = conveyorBeltPowerMsgRequest(0)
                        clientbelt(requestbelt)
                        ur5_2_posepackage = geometry_msgs.msg.Pose()
                        ur5_2_posepackage.position.x = ur5._pospkg_x
                        ur5_2_posepackage.position.y = ur5._pospkg_y - 0.05
                        ur5_2_posepackage.position.z = 1.19
                        # This to keep EE parallel to Ground Plane
                        ur5_2_posepackage.orientation.x = -0.5
                        ur5_2_posepackage.orientation.y = -0.5
                        ur5_2_posepackage.orientation.z = 0.5
                        ur5_2_posepackage.orientation.w = 0.5
                        ur5.go_to_pose(ur5_2_posepackage)
                        ur5.go_to_pose(ur5_2_posepackage)

                        request2 = vacuumGripperRequest(True)
                        client2(request2)
                        ur5.ee_cartesian_translation(0, 0, 0.5)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        ur5.go_to_pose(ur5_2_home_poseyellow)
                        requestbelt = conveyorBeltPowerMsgRequest(100)
                        clientbelt(requestbelt)
                        request2 = vacuumGripperRequest(False)
                        client2(request2)
                        ur5.received_msg[i] = "done"
                        ur5._pub2.publish(str(i))

                break


cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
