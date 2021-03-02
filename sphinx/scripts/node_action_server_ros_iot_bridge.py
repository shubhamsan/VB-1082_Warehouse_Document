#!/usr/bin/env python

"""
Action Client for the ROS IoT bridge, to receive and send requests and data to and from
the MQTT server.
"""

import rospy
import actionlib
import threading
import json
import requests

from std_msgs.msg import String
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pkg_ros_iot_bridge.msg import msgRosIotAction   
from pkg_ros_iot_bridge.msg import msgRosIotGoal 
from pkg_ros_iot_bridge.msg import msgRosIotResult   
from pkg_ros_iot_bridge.msg import msgRosIotFeedback
from pyzbar.pyzbar import decode
from pkg_ros_iot_bridge.msg import msgMqttSub 
#from pyiot import iot
import time
from datetime import timedelta,datetime
import datetime


class RosIotBridgeActionServer:

    """RosIotBridgeActionServer Class"""
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
            * self.on_goal - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Goal.

            * self.on_cancel - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._sub_topic = "/eyrc/vb/shamohsh/orders"
        print(param_config_iot)

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)

        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        mqttsub_onlineorder = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback_onlineorder,
                                                              self._config_mqtt_server_url,
                                                              self._config_mqtt_server_port,
                                                              self._sub_topic,
                                                              self._config_mqtt_qos)
        self._sub = rospy.Subscriber("chatter", String, callback=self.dispatched)
        self._sub2 = rospy.Subscriber("Talker", String, callback=self.shipped)

        self.msg = []
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
        self.enum = []
        self.color = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

        if (ret == 0):
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        if (mqttsub_onlineorder == 0):
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    def get_time_str(self):
        """ This is function to get current time """
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        # str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return value

    def dispatched(self, data):
        """This is a callback function dispatched order to send Data of Dispatched Order to OrdersDispatched Sheet"""
        s = data.data
        li = int(s)
        str_time = self.get_time_str().strftime('%Y-%m-%d %H:%M:%S')

        parameters = {"id": "OrdersDispatched", "Team Id": "VB#1082", "Unique Id": "shamohsh",
                      "Order ID": self.msg[li]["Order ID"],
                      "Item": self.msg[li]["Item"], "Priority": self.msg[li]["Priority"],
                      "City": self.msg[li]["City"],
                      "Dispatch Quantity": self.msg[li]["Order Quantity"], "Cost": self.msg[li]["Cost"],
                      "Dispatch Status": "Yes", "Dispatch Date and Time": str_time}
        URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
        response1 = requests.get(URL1, params=parameters)

    def shipped(self, data2):
        """This is a callback function of orders which are been shipped and to send Data of shipped Order to OrdersShipped Sheet"""
        s2 = data2.data
        li = int(s2)
        str_time = self.get_time_str().strftime('%Y-%m-%d %H:%M:%S')

        parameters = {"id": "OrdersShipped", "Team Id": "VB#1082", "Unique Id": "shamohsh",
                      "Order ID": self.msg[li]["Order ID"],
                      "Item": self.msg[li]["Item"], "Priority": self.msg[li]["Priority"],
                      "City": self.msg[li]["City"], "Estimated Time of Delivery": self.msg[li]["Date"],
                      "Shipped Quantity": self.msg[li]["Order Quantity"], "Cost": self.msg[li]["Cost"],
                      "Shipped Status": "Yes", "Shipped Date and Time": str_time}
        URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
        response1 = requests.get(URL1, params=parameters)

    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, message):
        """This is a callback function for MQTT Subscriptions"""
        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

    def mqtt_sub_callback_onlineorder(self, client, userdata, msg):
        """This is a call for mqtt subscriber and to Send Data of Incoming Order to IncomingOrders Sheet"""
        global priority, cost, Date
        payloadorder = str(msg.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payloadorder)
        print("[MQTT SUB CB] Topic: ", msg.topic)

        res = json.loads(payloadorder)
        if res["item"] == "Medicine":
            priority = "HP"
            cost = "450"
            Date = (self.get_time_str() + timedelta(1)).strftime('%Y-%m-%d')
        if res["item"] == "Food":
            priority = "MP"
            cost = "250"
            Date = (self.get_time_str() + timedelta(3)).strftime('%Y-%m-%d')
        if res["item"] == "Clothes":
            priority = "LP"
            cost = "150"
            Date = (self.get_time_str() + timedelta(5)).strftime('%Y-%m-%d')

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = msg.topic
        msg_mqtt_sub.message = priority

        self._handle_ros_pub.publish(msg_mqtt_sub)

        parameters = {"id": "IncomingOrders", "Team Id": "VB#1082", "Unique Id": "shamohsh",
                      "Order ID": res["order_id"], "Order Date and Time": res["order_time"],
                      "Item": res["item"], "Priority": priority, "Order Quantity": res["qty"], "City": res["city"],
                      "Longitude": res["lon"], "Latitude": res["lat"], "Cost": cost, "Date": Date}
        URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
        response1 = requests.get(URL1, params=parameters)

        self.msg = self.msg + [parameters]

    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        """This function will be called when Action Server receives a Goal"""
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if (goal.protocol == "mqtt"):

            if ((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()

                # Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return

        else:
            goal_handle.set_rejected()
            return

    # This function is called is a separate thread to process Goal.
    def process_goal(self, goal_handle):
        """This function is called is a separate thread to process Goal."""
        flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if (goal.protocol == "mqtt"):
            rospy.logwarn("MQTT")

            if (goal.mode == "pub"):
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if (ret == 0):
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif (goal.mode == "sub"):
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if (ret == 0):
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if (result.flag_success == True):
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        """This function will be called when Goal Cancel request is send to the Action Server"""
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()

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
                if self.color[0] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("R00")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines","Priority": "HP", "Storage Number": "R0 C0", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[0] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G00")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R0 C0", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[0] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y00")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R0 C0", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 313 < self.enum[i]["top"] < 320 and 312 < self.enum[i]["left"] < 320:
                self.color[1] = self.enum[i]["color"]
                if self.color[1] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("R01")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R0 C1", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[1] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G01")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R0 C1", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[1] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y01")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R0 C1", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 313 < self.enum[i]["top"] < 320 and 500 < self.enum[i]["left"] < 508:
                self.color[2] = self.enum[i]["color"]
                if self.color[2] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("R02")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R0 C2", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[2] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G02")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R0 C2", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response2 = requests.get(URL1, params=parameters)

                if self.color[2] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y02")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R0 C2", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 492 < self.enum[i]["top"] < 500 and 126 < self.enum[i]["left"] < 133:
                self.color[3] = self.enum[i]["color"]
                if self.color[3] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("R10")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R1 C0", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[3] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G10")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R1 C0", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[3] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y10")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R1 C0", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 492 < self.enum[i]["top"] < 500 and 312 < self.enum[i]["left"] < 320:
                self.color[4] = self.enum[i]["color"]
                if self.color[4] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("R11")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R1 C1", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[4] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G11")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R1 C1", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[4] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y11")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R1 C1", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 492 < self.enum[i]["top"] < 500 and 500 < self.enum[i]["left"] < 508:
                self.color[5] = self.enum[i]["color"]
                if self.color[5] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("R12")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R1 C2", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[5] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G12")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R1 C2", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[5] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y12")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R1 C2", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 642 < self.enum[i]["top"] < 648 and 126 < self.enum[i]["left"] < 133:
                self.color[6] = self.enum[i]["color"]
                if self.color[6] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("R20")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R2 C0", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[6] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G20")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R2 C0", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[6] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y20")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R2 C0", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 642 < self.enum[i]["top"] < 648 and 312 < self.enum[i]["left"] < 320:
                self.color[7] = self.enum[i]["color"]
                if self.color[7] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("R21")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R2 C1", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[7] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G21")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R2 C1", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[7] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y21")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R2 C1", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 642 < self.enum[i]["top"] < 648 and 500 < self.enum[i]["left"] < 508:
                self.color[8] = self.enum[i]["color"]
                if self.color[8] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU":"%s" % ("R22")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R2 C2", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[8] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G22")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R2 C2", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[8] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y22")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R2 C2", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 792 < self.enum[i]["top"] < 799 and 126 < self.enum[i]["left"] < 133:
                self.color[9] = self.enum[i]["color"]
                if self.color[9] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("R30")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R3 C0", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[9] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G30")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R3 C0", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[9] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y30")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R3 C0", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 792 < self.enum[i]["top"] < 799 and 312 < self.enum[i]["left"] < 320:
                self.color[10] = self.enum[i]["color"]
                if self.color[10] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU":"%s" % ("R31")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R3 C1", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[10] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G31")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R3 C1", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[10] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y31")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R3 C1", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

            if 792 < self.enum[i]["top"] < 799 and 500 < self.enum[i]["left"] < 508:
                self.color[11] = self.enum[i]["color"]
                if self.color[11] == "red":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("R32")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Medicines",
                                  "Priority": "HP", "Storage Number": "R3 C2", "Cost": "450", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[11] == "green":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("G32")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Clothes",
                                  "Priority": "LP", "Storage Number": "R3 C2", "Cost": "150", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

                if self.color[11] == "yellow":
                    parameters = {"id": "Inventory", "Team Id": "VB#1082", "Unique Id": "shamohsh", "SKU": "%s" % ("Y32")+ datetime.datetime.now().strftime('%m') + datetime.datetime.now().strftime('%y'),
                                  "Item": "Food",
                                  "Priority": "MP", "Storage Number": "R3 C2", "Cost": "250", "Quantity": "1"}
                    URL1 = "https://script.google.com/macros/s/AKfycbzBRhM2fJ0F-PM6LxqlxgCOb2Aa20p54J8C4XWli12mWBjsMw79t9Xa/exec"
                    response1 = requests.get(URL1, params=parameters)

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
        _,threshold = cv2.threshold(image, 70, 255, cv2.THRESH_TRUNC)
        self.get_qr_data(threshold)
        cv2.waitKey(3)


# Main
def main():
    """Main"""
    rospy.sleep(10)
    rospy.sleep(10)
    rospy.init_node('node_ros_iot_bridge_action_server')

    action_server = RosIotBridgeActionServer()

    rospy.spin()


if __name__ == '__main__':
    main()
