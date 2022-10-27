import numpy as np
import math

from dataclasses import dataclass
import time


import onnx
import onnxruntime as ort
import copy

#   ROS2 python 
import rclpy
from rclpy.node import Node
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import Timesync
from msg_srv_act_interface.srv import CollisionAvoidanceSetpoint

from sensor_msgs.msg import Image

from .JBNU_Obs import JBNU_Collision

# Opencv-ROS
import cv2


class NMPC_NET_Node(Node):
    def __init__(self):
        super().__init__('JBNU_module')
        print(" JBNU Module In ")
        ##  Output
        self.vel_cmd_x = 0.0
        self.vel_cmd_y = 0.0
        self.vel_cmd_z = 0.0
        self.vel_cmd_yaw = 0.0
        self.qosProfileGen()
        self.requestFlag = False
        self.response_timestamp = 0
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.requestFlag = False
        print(" JBNU Module Param init ")
        self.model_pretrained = onnx.load('/root/ros_ws/src/a4vai/a4vai/collision_avoidance/feedforward.onnx')
        print(onnx.checker.check_model(self.model_pretrained))
        self.sess = ort.InferenceSession(self.model_pretrained.SerializeToString(),providers=ort.get_available_providers())
        print(" JBNU Module Model load complete ")
        self.CameraSubscriber_ = self.create_subscription(Image, '/realsense_d455_depth/image', self.CameraCallback, QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.JBNUmoduleService_ = self.create_service(CollisionAvoidanceSetpoint, 'collision_avoidance', self.CollisionAvoidanceCallback)
        
    def qosProfileGen(self):
    #   Reliability : 데이터 전송에 있어 속도를 우선시 하는지 신뢰성을 우선시 하는지를 결정하는 QoS 옵션
    #   History : 데이터를 몇 개나 보관할지를 결정하는 QoS 옵션
    #   Durability : 데이터를 수신하는 서브스크라이버가 생성되기 전의 데이터를 사용할지 폐기할지에 대한 QoS 옵션
        self.QOS_Sub_Sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        self.QOS_Service = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        
    def CollisionAvoidanceCallback(self, request, response):
        print("===== Request Coliision Avoidance Node =====")
        self.requestFlag = request.request_collisionavoidance
        self.requestTimestamp = request.request_timestamp
        if self.requestFlag is True : 
            self.vel_cmd_x, self.vel_cmd_y, self.vel_cmd_z, self.vel_cmd_yaw = JBNU_Collision.CA(self.current_frame, self.sess)
            print("===== Coliision Avoidance Complete!! =====")
            response.response_timestamp = self.response_timestamp
            response.response_collisionavoidance = True
            response.vel_cmd_x = self.vel_cmd_x
            response.vel_cmd_y = self.vel_cmd_y
            response.vel_cmd_z = self.vel_cmd_z
            response.vel_cmd_yaw = self.vel_cmd_yaw
            print("===== Response Coliision Avoidance Node =====")
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_collisionavoidance = False
            response.vel_cmd_x = 0
            response.vel_cmd_y = 0
            response.vel_cmd_z = 0
            response.vel_cmd_y = 0
            return response
        
    def TimesyncCallback(self, msg):
        self.response_timestamp = msg.timestamp
        
    def CameraCallback(self, msg):
        self.current_frame = self.CvBridge.imgmsg_to_cv2(msg)
