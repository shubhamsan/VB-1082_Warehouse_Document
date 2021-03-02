#! /usr/bin/env python
"""This script controls the UR5 arm (UR5_1) near the shelf for placing boxes on the conveyor"""
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import requests
from pkg_ros_iot_bridge.msg import *  # Message Class that is used for Result Messages
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import tf2_ros
import yaml
import os
import math
import time
import sys
import copy
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest

from std_srvs.srv import Empty


class Ur5Moveit:
    """Class for Ur5Moveit"""

    # Constructor
    def __init__(self, arg_robot_name):
        """Initialize the Constructor"""

        rospy.init_node('task6node1', anonymous=True)
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
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        self.received_msg = []
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
        self.sub = rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, self.sub_callback)
        self.enum = []
        self.color = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        self._pub = rospy.Publisher("chatter", String, queue_size=10)

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def sub_callback(self, msg):
        """This is a callback function of  MQTT Subscriptions /ros_iot_bridge/mqtt/sub"""
        self.received_msg = self.received_msg + [msg.message]

    def clear_octomap(self):
        """Clear Octomap"""
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

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

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        """This Function forces or call set_joint_angles multiple time to hardset the joint of ur5 arm"""
        number_attempts = 0
        flag_success = False
        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """Load yaml file and execute them
            Parameters:
            arg_file_path (str): path of folder where saved trajectry stored
            arg_file_name (str): file name
                """
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open, Loader=yaml.Loader)

        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        """This Function plays moveit_play_planned_path_from_file multiple times"""
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
        # # self.clear_octomap()

        return True

    def get_qr_data(self, arg_image):
        """
            Get the data From image and process Data.
            Send Inventory status to Inventory Sheet
        """
        global e
        qr_result = decode(arg_image)
        # initializing list
        for i in range(0, len(qr_result)):
            a = qr_result[i].data
            b = qr_result[i].rect[0]
            c = qr_result[i].rect[1]
            e = {"left": b, "top": c, "color": a}
            self.enum = self.enum + [e]

        self.enum.sort(key=lambda x: x.get('left'))

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

        print self.color

    def callback(self, data):
        """This is a callback function of subscriber /eyrc/vb/camera_1/image_raw """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_sub.unregister()

        except CvBridgeError as e:
            rospy.logerr(e)
        (rows, cols, channels) = cv_image.shape
        result = cv2.fastNlMeansDenoisingColored(cv_image, None, 20, 10, 7, 21)
        image = cv_image
        # Resize a 720x1280 image to 360x640 to fit it on the screen
        _, threshold = cv2.threshold(image, 70, 255, cv2.THRESH_TRUNC)
        self.get_qr_data(threshold)
        cv2.waitKey(3)

    # Destructor"""

    def __del__(self):
        """Destructor"""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    """Main"""

    rospy.sleep(10)
    rospy.sleep(10)
    ur5 = Ur5Moveit(sys.argv[1])
    rospy.sleep(10)
    rospy.sleep(10)
    rospy.sleep(10)
    rospy.sleep(10)

    rospy.wait_for_service("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1")
    client = rospy.ServiceProxy("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1", vacuumGripper)

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1.20
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    lst_joint_angles_home = [math.radians(172),
                             math.radians(-40),
                             math.radians(58),
                             math.radians(-108),
                             math.radians(-90),
                             math.radians(-7.8)]

    lst_joint_angles_straight = [math.radians(0),
                                 math.radians(-90),
                                 math.radians(0),
                                 math.radians(0),
                                 math.radians(0),
                                 math.radians(0)]
    lst_joint_angles_straight_180 = [math.radians(180),
                                     math.radians(-90),
                                     math.radians(0),
                                     math.radians(0),
                                     math.radians(0),
                                     math.radians(0)]
    lst_joint_angles_1 = [math.radians(-57),
                          math.radians(-66),
                          math.radians(-5),
                          math.radians(-103),
                          math.radians(-116),
                          math.radians(-83)]
    lst_joint_angles_1_back = [math.radians(-38),
                               math.radians(-64),
                               math.radians(-23),
                               math.radians(-86),
                               math.radians(-135),
                               math.radians(-80)]
    lst_joint_angles_2 = [math.radians(-123),
                          math.radians(-62),
                          math.radians(-31),
                          math.radians(-87.7),
                          math.radians(-57),
                          math.radians(0)]
    lst_joint_angles_2_back = [math.radians(-171),
                               math.radians(-108),
                               math.radians(-40),
                               math.radians(-113),
                               math.radians(-9),
                               math.radians(0)]
    lst_joint_angles_3 = [math.radians(49),
                          math.radians(-112),
                          math.radians(-8),
                          math.radians(-58),
                          math.radians(132),
                          math.radians(0)]
    lst_joint_angles_3_back = [math.radians(35),
                               math.radians(-100),
                               math.radians(-28),
                               math.radians(-50),
                               math.radians(145),
                               math.radians(0)]
    lst_joint_angles_4 = [math.radians(-57),
                          math.radians(-82),
                          math.radians(34),
                          math.radians(50),
                          math.radians(121),
                          math.radians(1)]

    lst_joint_angles_4_back = [math.radians(-40),
                               math.radians(-91),
                               math.radians(43),
                               math.radians(51),
                               math.radians(137),
                               math.radians(1)]
    lst_joint_angles_5 = [math.radians(125),
                          math.radians(-81),
                          math.radians(-46),
                          math.radians(126),
                          math.radians(-55),
                          math.radians(0)]
    lst_joint_angles_5_back = [math.radians(173),
                               math.radians(-67),
                               math.radians(-52),
                               math.radians(116),
                               math.radians(-8),
                               math.radians(4)]
    lst_joint_angles_5_back_2 = [math.radians(152),
                                 math.radians(-68),
                                 math.radians(-48),
                                 math.radians(-66),
                                 math.radians(40),
                                 math.radians(2)]

    lst_joint_angles_6 = [math.radians(48),
                          math.radians(-158),
                          math.radians(70),
                          math.radians(-90),
                          math.radians(133),
                          math.radians(0)]
    lst_joint_angles_6_1 = [math.radians(49),
                            math.radians(-92),
                            math.radians(-69),
                            math.radians(-17),
                            math.radians(133),
                            math.radians(0)]

    lst_joint_angles_6_back = [math.radians(37),
                               math.radians(-86),
                               math.radians(-75),
                               math.radians(-16),
                               math.radians(145),
                               math.radians(0)]

    lst_joint_angles_7 = [math.radians(-56),
                          math.radians(-99),
                          math.radians(88),
                          math.radians(11),
                          math.radians(121),
                          math.radians(0)]
    lst_joint_angles_7_back = [math.radians(-35),
                               math.radians(-109),
                               math.radians(96),
                               math.radians(13),
                               math.radians(142),
                               math.radians(0)]
    lst_joint_angles_8 = [math.radians(116),
                          math.radians(-63),
                          math.radians(-96),
                          math.radians(159),
                          math.radians(-64),
                          math.radians(0)]
    lst_joint_angles_8_back = [math.radians(155),
                               math.radians(-44),
                               math.radians(-101),
                               math.radians(148),
                               math.radians(-25),
                               math.radians(0)]

    lst_joint_angles_9 = [math.radians(48),
                          math.radians(-89),
                          math.radians(-77),
                          math.radians(166),
                          math.radians(-132),
                          math.radians(0)]
    lst_joint_angles_9_back = [math.radians(37),
                               math.radians(-83),
                               math.radians(-82),
                               math.radians(166),
                               math.radians(-143),
                               math.radians(0)]

    lst_joint_angles_10 = [math.radians(-59),
                           math.radians(-97),
                           math.radians(118),
                           math.radians(-21),
                           math.radians(118),
                           math.radians(1)]
    lst_joint_angles_10_back = [math.radians(-45),
                                math.radians(-107),
                                math.radians(126),
                                math.radians(-18),
                                math.radians(132),
                                math.radians(1)]
    lst_joint_angles_11 = [math.radians(-126),
                           math.radians(-119),
                           math.radians(133),
                           math.radians(-14),
                           math.radians(51),
                           math.radians(0)]
    lst_joint_angles_12 = [math.radians(53),
                           math.radians(-86),
                           math.radians(-115),
                           math.radians(-159),
                           math.radians(-127),
                           math.radians(0)]
    lst_joint_angles_12_back = [math.radians(36),
                                math.radians(-77),
                                math.radians(-122),
                                math.radians(-160),
                                math.radians(-144),
                                math.radians(0)]

    while not rospy.is_shutdown():
        for i in range(len(ur5.received_msg)):
            if ((ur5.received_msg[i] == "HP") and (ur5.color[0] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_1, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_1_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[0] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "HP") and (ur5.color[1] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_2, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_2_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[1] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "HP") and (ur5.color[2] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_3, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_3_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[2] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "HP") and (ur5.color[3] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_4, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_4_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[3] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "HP") and (ur5.color[4] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose5.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose5_to_pose5back.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[4] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "HP") and (ur5.color[5] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_6_1, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_6_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[5] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "HP") and (ur5.color[6] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_7, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_7_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[6] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "HP") and (ur5.color[7] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose8.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_8_back, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose8back_to_straight.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[7] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "HP") and (ur5.color[8] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose9.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose9_to_pose9back.yaml', 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose9back_to_straight.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[8] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "HP") and (ur5.color[9] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_10, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.ee_cartesian_translation(0, 0.5, 0)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[9] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "HP") and (ur5.color[11] == "red")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose12.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose12_to_pose12back.yaml', 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose12back_to_straight.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[11] = "done"
            if ((ur5.received_msg[i] == "MP") and (ur5.color[0] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_1, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_1_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[0] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "MP") and (ur5.color[1] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_2, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_2_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[1] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "MP") and (ur5.color[2] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_3, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_3_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[2] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "MP") and (ur5.color[3] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_4, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_4_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[3] = "done"

            if ((ur5.received_msg[i] == "MP") and (ur5.color[5] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_6_1, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_6_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[5] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "MP") and (ur5.color[6] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_7, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_7_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[6] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "MP") and (ur5.color[7] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose8.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_8_back, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose8back_to_straight.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[7] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "MP") and (ur5.color[8] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose9.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose9_to_pose9back.yaml', 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose9back_to_straight.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[8] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "MP") and (ur5.color[9] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_10, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.ee_cartesian_translation(0, 0.5, 0)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[9] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "MP") and (ur5.color[11] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose12.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose12_to_pose12back.yaml', 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose12back_to_straight.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[11] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "MP") and (ur5.color[4] == "yellow")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose5.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_5_back, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose5back_to_straight.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[4] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[0] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_1, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_1_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[0] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[1] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_2, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_2_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[1] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[2] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_3, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_3_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[2] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[3] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_4, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_4_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[3] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[4] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose5.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose5_to_pose5back.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[4] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[5] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_6_1, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_6_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[5] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[6] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_7, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_7_back, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[6] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[7] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose8.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.hard_set_joint_angles(lst_joint_angles_8_back, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose8back_to_straight.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[7] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[8] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose9.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose9_to_pose9back.yaml', 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose9back_to_straight.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[8] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[9] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_10, 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.ee_cartesian_translation(0, 0.5, 0)
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[9] = "done"
                ur5._pub.publish(str(i))
            if ((ur5.received_msg[i] == "LP") and (ur5.color[11] == "green")):
                ur5.hard_set_joint_angles(lst_joint_angles_straight, 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'straight_to_pose12.yaml', 5)
                request = vacuumGripperRequest(True)
                client(request)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose12_to_pose12back.yaml', 5)
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pose12back_to_straight.yaml', 5)
                ur5.hard_set_joint_angles(lst_joint_angles_home, 5)
                request = vacuumGripperRequest(False)
                client(request)
                ur5.received_msg[i] = "done"
                ur5.color[11] = "done"
                ur5._pub.publish(str(i))


cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
