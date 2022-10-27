import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import math
from itertools import chain

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
from rclpy.qos import ReliabilityPolicy, QoSProfile, LivelinessPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge


from pytictoc import TicToc


#   Gazebo Client Reset, Pause, Unpase, SRV
from std_srvs.srv import Empty

#   PX4 MSG - Sub
from px4_msgs.msg import EstimatorStates
from px4_msgs.msg import VehicleAngularVelocity
#   PX4 MSG - Pub
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleAttitudeSetpoint
from rclpy.qos import ReliabilityPolicy, QoSProfile, LivelinessPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


from .map_service import MapService
#from .map_queue.map_queue import M
from .path_plan_service import PathPlanningService
from .path_follow_service import PathFollowingGPRService, PathFollowingService, PathFollowingGuidService
from.collision_avoidance_service import CollisionAvoidanceService

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')
        self.initializeParameter()
        self.qosProfileGen()
        self.declare_publisher_px4()
        self.declare_subscriber_px4()
        self.declare_service_gazebo()
        #self.RequestMapGeneration()
        
        # Offboard Period
        self.TestTimer = self.create_timer(self.TestPeriod, self.TestCallback)

        self.AlgorithmTimer = self.create_timer(self.AlgorithmPeriod, self.AlgorithmCallback)
        self.OffboardTimer_attitude = self.create_timer(self.OffboardPeriod_AttCmd, self.OffboardControl_AttCmd)
        self.OffboardTimer_velocity = self.create_timer(self.OffboardPeriod_VelCmd, self.OffboardControl_VelCmd)
        
    def initializeParameter(self):
        self.timestamp_offboard = 0
        self.initTimeStamp_GPR = 0
        self.initTimeStamp_PF_Att_Cmd = 0
        self.initFlag_GPR = True
        self.initFlag_PF_Att_Cmd = True


        self.requestFlag = False
        # Vehicle States Variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        
        self.ObsAngle = 0.0
        self.ObsSize  = 0.0
        self.ObsPos = [0.0, 0.0]
        
        ############# vel cmd ###################
        self.vel_cmd_x = 0.0
        self.vel_cmd_y = 0.0
        self.vel_cmd_z = 0.0
        self.vel_cmd_yaw = 0.0
        
        self.z_NDO_past = []


        # Controller Period
        self.AlgorithmPeriod = 1/50
        self.OffboardPeriod_AttCmd = 1/250
        self.OffboardPeriod_VelCmd = 1/50
        self.OffboardPeriod_PosCmd = 1/50
        self.TestPeriod = 1/100
        
        # Armming Flag
        self.ArmmingCounter = 100
        
        # Arm Disarm Command
        self.VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
        # Offboard Mode Command
        self.VEHICLE_CMD_DO_SET_MODE = 176
        
        
        ## Controller Sample Variables
        self.TargetPositionCmd = [0.0, 0.0, 0.0] # Meter
        self.TargetVelocityCmd = [0.0, 0.0, 0.0] # Meter
        self.TargetAttitudeCmd = [0.0, 0.0, 0.0, 0.0] # Quaternion w x y z
        self.TargetRateCmd = [0.0, 0.0, 0.0] # Radian
        self.TargetThrustCmd = 0.0
        self.TargetBodyRateCmd = [np.NaN, np.NaN, np.NaN]
        self.TargetYawRateCmd = 0.0
        
        #############################
        #####   Sequence Flag   #####
        #############################
        self.map_generation_flag = True
        self.path_planning_complete = False
        self.InitialPositionFlag = False
        self.InitialPosition = [0.0, 0.0, -5.0]

        
        self.start_point = [0.0, 0.0]
        self.goal_point = [4999.0, 4999.0]
        
        self.PP_response_timestamp = 0
        
        self.path_planning_flag = False
        self.path_planning_complete = False
        
        self.path_following_flag = False
        self.path_following_complete = False

        self.path_following_gpr_flag = False
        self.path_following_gpr_complete = False
        
        self.path_following_guid_flag = False
        self.path_following_guid_complete = False
        
        
        self.PF_response_setpoint_timestamp = 0
        self.PF_response_gpr_timestamp = 0
        self.PF_response_guid_timestamp = 0
        
        # self.start_point = []
        # self.goal_point = []
        
        self.collision_avoidance_flag = True
        self.collision_avoidance_complete = False
        
        self.OffboardCount = 0
        self.OffboardCounter = 1000
        
        self.waypoint_x = []
        self.waypoint_y = []
        self.waypoint_z = []
        self.waypoint_index = 0
        
        self.TargetThrust = []
        self.TargetPosition = []
        # self.TargetAttitude = []
        self.TargetYaw = 0.0
        
        self.LAD = 0.0
        self.SPDCMD = 0.0
        self.outNDO = [0.0, 0.0, 0.0]
        self.gpr_output = []
        self.gpr_output_data = []
        self.gpr_output_index = 0
        self.flag_guid_param = 1

        self.request_path_plnning_flag = True
        
        self.declare_subscriber_gazebo()
        
        ################################
        self.waypoint_x = [194.50006744625216, 825.8771283644521, 1431.8613353993596, 2000.3352614176797, 2397.700009556139, 2730.8000037362776, 2706.3233754062016, 
                           3225.794498676967, 3627.9853373120645, 4127.434997699735, 4470.346028786243]
        self.waypoint_y = [36.118291873152884, 144.17673246974937, 309.82613153979173, 537.4419799797172, 922.1794141237731, 1360.298574161542, 2178.636181251442,
                           2407.6458042137847, 2782.003343801756, 3054.981891820806,  3482.83100675065]
        self.waypoint_z = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
        # self.wapoint_index = 11
        self.gpr_output = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],
                           [0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],
                           [0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
        
        # self.gpr_output_data = list(chain.from_iterable(self.gpr_output))
        self.gpr_output_index = 3
        self.outNDO = [0.0,0.0,0.0]

        ######################################
        
        
#################################################################################################################
    def TestCallback(self):
        if self.OffboardCount == self.OffboardCounter:
            self.offboard()
            self.arm()
                
        self.OffboardControlModeCallback()

        if self.InitialPositionFlag == True:
            self.get_logger().info("===== Test Timer On =====")
            
            if self.map_generation_flag is True :
                map_service = MapService()
                map_service.RequestMapGeneration(self.map_generation_flag)
                rclpy.spin_once(map_service)
                if map_service.future.done():
                    try : 
                        map_service.result = map_service.future.result()
                    except Exception as e:
                        map_service.get_logger().info(
                            'Service call failed %r' % (e,))
                    else :
                        map_service.get_logger().info( "Map Generation Complete!! ")
                        print(map_service.result.map_sequence_init)
                        if map_service.result.map_sequence_init is True :
                            self.map_generation_flag = False
                            self.path_planning_flag = True
                        else :
                            pass    
                    finally : 
                        map_service.destroy_node()
            else : 
                pass
            ##     Path Planning MODULE
            if self.path_planning_flag is True :
                
                planning_service = PathPlanningService()
                planning_service.RequestPathPlanning(self.start_point, self.goal_point)
                rclpy.spin_until_future_complete(planning_service, planning_service.future)
                if planning_service.future.done():
                    try : 
                        planning_service.result = planning_service.future.result()
                    except Exception as e:
                        planning_service.get_logger().info(
                            'Path Planning Service call failed %r' % (e,))
                    else :
                        planning_service.get_logger().info( "Path Planning Complete!! ")
                        if planning_service.result.response_pathplanning is True :
                            self.PP_response_timestamp = planning_service.result.response_timestamp
                            self.waypoint_x = planning_service.result.waypoint_x
                            self.waypoint_y = planning_service.result.waypoint_y
                            self.waypoint_z = planning_service.result.waypoint_z
                            self.path_planning_flag = False
                            self.path_planning_complete = True
                            self.path_following_guid_flag = True
                        else :
                            pass
                    finally : 
                        planning_service.destroy_node()
                else : 
                    print("Can't Path plan")
            else : 
                pass
            
            if np.sqrt((self.x - self.waypoint_x[self.waypoint_index])**2 + (self.y - self.waypoint_y[self.waypoint_index]) ** 2) < 3.0:
                self.waypoint_index += 1
                print("waypoint_index", str(self.waypoint_index))
            else : 
                print("else function-----------------------------------")
        else:
            self.Takeoff()
            
        if self.OffboardCount < self.OffboardCounter:
            self.OffboardCount = self.OffboardCount + 1

        # ###     PF GPR MODULE
        # if self.path_following_gpr_flag is True :
        #     if self.initFlag_GPR is True : 
        #         self.initTimeStamp_GPR = self.timestamp_offboard
        #         self.initFlag_GPR = False
        #     print("===== Path Following GPR Sequence =====")
        #     following_gpr_service = PathFollowingGPRService()
        #     print(self.initTimeStamp_GPR)
        #     following_gpr_service.RequestPathFollowingGPR(self.outNDO, self.initTimeStamp_GPR)
        #     print("===== Debug point 1 =====")

        #     # rclpy.spin_once(following_gpr_service)
        #     rclpy.spin_until_future_complete(following_gpr_service, following_gpr_service.future_gpr)
        #     if following_gpr_service.future_gpr.done():
        #         try : 
        #             following_gpr_service.result = following_gpr_service.future_gpr.result()
        #         except Exception as e:
        #             following_gpr_service.get_logger().info(
        #                 'Service call failed %r' % (e,))
        #         else :
        #             following_gpr_service.get_logger().info( "Path Following GPR Module Complete!! ")
        #             if following_gpr_service.result.response_gpr is True :
        #                 self.PF_response_gpr_timestamp = following_gpr_service.result.response_timestamp
        #                 self.gpr_output_data = following_gpr_service.result.gpr_output_data
        #                 self.gpr_output_index = following_gpr_service.result.gpr_output_index
        #                 self.gpr_output = [self.gpr_output_data[i * self.gpr_output_index:(i + 1) * self.gpr_output_index] for i in range((len(self.gpr_output_data) - 1 + self.gpr_output_index) // self.gpr_output_index )]
        #                 self.path_following_gpr_flag = False
        #                 print(following_gpr_service.result.response_timestamp)
        #                 self.path_following_gpr_complete = True
        #             else :
        #                 pass    
        #         finally : 
        #             following_gpr_service.destroy_node()
        #     else : 
        #         self.get_logger().warn("===== Path Following GPR Module Can't Response =====")
        # else : 
        #     pass
        
        # if self.path_following_gpr_complete is True : 
        #     print("gpr_output = ", str(self.gpr_output))
        #     print("gpr_output_index = ", str(self.gpr_output_index))
        #     self.path_following_gpr_complete = False
        
       
        
        
        
        
        
    def AlgorithmCallback(self):
    
            
        self.get_logger().info("===== AlgorithmCallback Timer On =====")
        ##     PF GUID MODULE
        if self.path_following_guid_flag is True :
            following_guid_service = PathFollowingGuidService()
            following_guid_service.RequestPathFollowingGuid(self.waypoint_x, self.waypoint_y, self.waypoint_z, self.waypoint_index, self.gpr_output_data, self.gpr_output_index, self.outNDO, self.flag_guid_param)

            # rclpy.spin_once(following_guid_service)
            rclpy.spin_until_future_complete(following_guid_service, following_guid_service.future_guid)
            if following_guid_service.future_guid.done():
                try : 
                    following_guid_service.result = following_guid_service.future_guid.result()
                except Exception as e:
                    following_guid_service.get_logger().info(
                        'Service call failed %r' % (e,))
                else :
                    following_guid_service.get_logger().info( "Path Following Guid Module Complete!! ")
                    print(following_guid_service.result.response_guid)
                    if following_guid_service.result.response_guid is True :
                        self.PF_response_guid_timestamp = following_guid_service.result.response_timestamp
                        self.LAD = following_guid_service.result.lad
                        self.SPDCMD = following_guid_service.result.spd_cmd
                        # self.path_following_guid_flag = False
                        print(following_guid_service.result.response_timestamp)
                        self.path_following_guid_complete = True
                        self.path_following_flag = True
                    else :
                        pass    
                finally : 
                    following_guid_service.destroy_node()
            else :
                self.get_logger().warn("===== Path Following Guid Module Can't Response =====")
        else : 
            pass
        
        if self.path_following_guid_complete is True : 
            print("LAD = ",str(self.LAD))
            print("SPDCMD = ", str(self.SPDCMD))


        
        

    def OffboardControl_AttCmd(self):
         if self.requestFlag == False :
            # OffboardTimer_velocity.destory()
            self.get_logger().info("===== Attitude Cmd Timer On =====")
            ###  PF Attitude Command Module(Setpoint)
            if self.path_following_flag is True :
                if self.initFlag_PF_Att_Cmd is True :
                    self.initTimeStamp_PF_Att_Cmd = self.timestamp_offboard
                    print("test")
                    self.initFlag_PF_Att_Cmd = False
                    
                following_service = PathFollowingService()
                print("initTimeStamp = ", str(self.initTimeStamp_PF_Att_Cmd))
                following_service.RequestPathFollowing(self.waypoint_x, self.waypoint_y, self.waypoint_z, self.waypoint_index, self.LAD, self.SPDCMD, self.initTimeStamp_PF_Att_Cmd, self.z_NDO_past)
                # rclpy.spin_once(following_service)
                rclpy.spin_until_future_complete(following_service, following_service.future_setpoint)
                if following_service.future_setpoint.done():
                    try : 
                        following_service.result = following_service.future_setpoint.result()
                    except Exception as e:
                        following_service.get_logger().info(
                            'Service call failed %r' % (e,))
                    else :
                        following_service.get_logger().info( "Path Following Setpoint Module Complete!! ")
                        print(following_service.result.response_pathfollowing)
                        if following_service.result.response_pathfollowing is True :
                            self.PF_response_setpoint_timestamp = following_service.result.response_timestamp
                            self.TargetThrust = following_service.result.targetthrust
                            self.TargetThrustCmd = self.TargetThrust
                            self.TargetAttitude = following_service.result.targetattitude
                            w, x, y, z  =   self.Euler2Quaternion(self.TargetAttitude[0], self.TargetAttitude[1], self.TargetAttitude[2])
                            self.TargetAttitudeCmd = np.array([w, x, y, z])
                            self.TargetPosition = following_service.result.targetposition
                            self.TargetYaw = following_service.result.targetyaw
                            self.outNDO = following_service.result.out_ndo
                            self.z_NDO_past = following_service.result.z_ndo
                            # self.path_following_flag = False
                            # print(following_service.result.response_timestamp)
                            self.path_following_complete = True
                        else :
                            pass    
                    finally : 
                        pass
                        #following_service.destroy_node()
                else : 
                    self.get_logger().warn("===== Path Following Setpoint Module Can't Response =====")
            else : 
                pass
            
            if self.path_following_complete is True : 
                print("TargetThrust = ", str(self.TargetThrust))
                print("TargetAttitude = ", str(self.TargetAttitude))
                print("TargetPosition = ", str(self.TargetPosition))
                print("TargetYaw = ", str(self.TargetYaw))
                print("outNDO = ", str(self.outNDO))
                self.SetAttitude([w, x, y, z], [0.0,0.0,0.0], self.TargetThrust, self.TargetYaw)

            
    def OffboardControl_VelCmd(self):
         if self.requestFlag == True :
            OffboardTimer_attitude.destroy()

            self.get_logger().info("===== Velocity Cmd Timer On =====")
            ### Collision Avoidance Module
            if self.collision_avoidance_flag is True :
                print("===== Collision Avoidance Sequence =====")
                collision_avoidance_service = CollisionAvoidanceService()
                collision_avoidance_service.RequestCollisionAvoidance()
                print("===== Debug point 1 =====")
                rclpy.spin_until_future_complete(collision_avoidance_service, collision_avoidance_service.future)

                print("===== Debug point 2=====")
                if collision_avoidance_service.future.done():
                    print("===== Debug point 3 =====")
                    try : 
                        collision_avoidance_service.result = collision_avoidance_service.future.result()
                    except Exception as e:
                        collision_avoidance_service.get_logger().info(
                            'Service call failed %r' % (e,))
                    else :
                        collision_avoidance_service.get_logger().info( "Collision Avoidance Module Complete!! ")
                        print(collision_avoidance_service.result.response_collisionavoidance)
                        if collision_avoidance_service.result.response_collisionavoidance is True :
                            self.response_timestamp = collision_avoidance_service.result.response_timestamp
                            self.vel_cmd_x = collision_avoidance_service.result.vel_cmd_x
                            self.vel_cmd_z = collision_avoidance_service.result.vel_cmd_z
                            self.vel_cmd_yaw = collision_avoidance_service.result.vel_cmd_yaw
                            
                            #self.collision_avoidance_flag = True
            
                            # print(collision_avoidance_service.result.response_timestamp)
                            self.collision_avoidance_complete = True
                            print("vel_cmd_x = ", str(self.vel_cmd_x))
                            print("vel_cmd_z = ", str(self.vel_cmd_z))
                            print("vel_cmd_yaw = ", str(self.vel_cmd_yaw))
                            self.SetVelocity([self.vel_cmd_x, 0.0, self.vel_cmd_z], self.vel_cmd_yaw)
                        else :
                            pass    
                    finally : 
                        pass
                        #collision_avoidance_service.destroy_node()
                else :
                    self.get_logger().warn("===== Collision Avoidance Module Can't Response =====")
            else : 
                pass
        

        
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
        
        #   센서와 같이 지속성이 높으며 데이터 유실의 문제보다는 순간적인 데이터를 가장 빠르게 전달해야하는 sensor data의 경우에는 아래와 같이 설정
        self.QOS_Pub_FastCmd = qos_profile_sensor_data
        print("=== QOS Profile Generation ===")
        
    def declare_publisher_px4(self):
        #   init PX4 MSG Publisher
        self.VehicleCommandPublisher_ = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', self.QOS_Sub_Sensor)
        self.OffboardControlModePublisher_ = self.create_publisher(OffboardControlMode, '/fmu/offboard_control_mode/in', self.QOS_Sub_Sensor)
        self.TrajectorySetpointPublisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', self.QOS_Sub_Sensor)
        self.VehicleAttitudeSetpointPublisher_ = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/vehicle_attitude_setpoint/in', self.QOS_Sub_Sensor)
        #self.VehicleRatesSetpointPublisher_ = self.create_publisher(VehicleRatesSetpoint, '/fmu/vehicle_rates_setpoint/in', self.QOS_Sub_Sensor)
        print("====== px4 Publisher Open ======")
        
    def declare_subscriber_px4(self):
        #   init PX4 MSG Subscriber
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.EstimatorStatesSubscriber_ = self.create_subscription(EstimatorStates, '/fmu/estimator_states/out', self.EstimatorStatesCallback, self.QOS_Sub_Sensor)
        self.VehicleAngularVelocitySubscriber_ = self.create_subscription(VehicleAngularVelocity, '/fmu/vehicle_angular_velocity/out', self.VehicleAngularVelocityCallback, self.QOS_Sub_Sensor)
        print("====== px4 Subscriber Open ======")
        
    def declare_subscriber_gazebo(self):
        # Init Camera Subscriber
        # self.CameraSubscriber_ = self.create_subscription(Image, '/realsense_d455_RGB/image', self.CameraCallback, self.QOS_Sub_Sensor)
        # Init Lidar Subscriber
        self.LidarSubscriber_ = self.create_subscription(LaserScan, '/rplidar_a3/laserscan', self.LidarCallback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        print("====== gazebo Subscriber Open ======")    

    def declare_service_gazebo(self):
        # init Gazebo Client
        self.ResetWorldClient = self.create_client(Empty, '/reset_world')
        self.ResetWorldClientRequest = Empty.Request()
        # while not self.ResetWorldClient.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('service not available, waiting again...')
        self.PauseClient = self.create_client(Empty, '/pause_physics')
        self.PauseClientRequest = Empty.Request()
        self.UnpauseClient = self.create_client(Empty, '/unpause_physics')
        self.UnpauseClientRequest = Empty.Request()
        print("====== gazebo Service Open ======")    


    def TimesyncCallback(self, msg):
        self.timestamp_offboard = msg.timestamp
        
    def EstimatorStatesCallback(self, msg):
        #   TimeStamp
        self.EstimatorStatesTime = msg.timestamp
        #   Position NED
        self.x = msg.states[7]
        self.y = msg.states[8]
        self.z = msg.states[9]
        #   Velocity NED
        self.vx = msg.states[4]
        self.vy = msg.states[5]
        self.vz = msg.states[6]
        #   Attitude
        self.phi, self.theta, self.psi = self.Quaternion2Euler(msg.states[0], msg.states[1], msg.states[2], msg.states[3])
        #   Wind Velocity NE
        self.wn = msg.states[22]
        self.we = msg.states[23]

    # VehicleAngularVelocity
    def VehicleAngularVelocityCallback(self, msg):
        # Rate
        self.p = msg.xyz[0] * 57.295779513
        self.q = msg.xyz[1] * 57.295779513
        self.r = msg.xyz[2] * 57.295779513
        
    # OffboardControlMode
    def OffboardControlModeCallback(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_offboard
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = True
        msg.body_rate = True
        self.OffboardControlModePublisher_.publish(msg)

    ## Vehicle Mode
    # Arming
    def arm(self):
        self.VehicleCommandCallback(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)

    # Disarming
    def disarm(self):
        self.VehicleCommandCallback(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)

    # Offboard
    def offboard(self):
        self.VehicleCommandCallback(self.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def VehicleCommandCallback(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_offboard
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        self.VehicleCommandPublisher_.publish(msg)
    
    ##  Gazebo Client
    #   Empty
    def SendResetWorld(self):
        self.ResetWorldClient.call_async(self.ResetWorldClientRequest)
    #   Pause
    def SendPause(self):
        self.PauseClient.call_async(self.PauseClientRequest)
    #   Unpause
    def SendUnpause(self):
        self.UnpauseClient.call_async(self.UnpauseClientRequest)
        
    ## Gazebo User Level Fucntion
    # Gazebo Reset 
    def Reset(self):
        self.SendResetWorld()

    # Gazebo Pause
    def Pause(self):
        self.SendPause()

    # Gazebo Unpause
    def Unpause(self):
        self.SendUnpause()
        
    ## PX4 User Level Function
    # Takeoff
    def Takeoff(self):
        self.SetPosition(self.InitialPosition, 0.0)
        if abs(self.z - self.InitialPosition[2]) < 0.3:
            self.InitialPositionFlag = True
        
    ## PX4 Controller
    # Set Position
    def SetPosition(self, SetPosition, SetYaw):
        SetVelocity = [np.NaN, np.NaN, np.NaN]
        self.TrajectorySetpointCallback(SetPosition, SetVelocity, SetYaw)
        
    # Set Velocity
    def SetVelocity(self, SetVelocity, SetYaw):
        SetPosition = [np.NaN, np.NaN, np.NaN]
        self.TrajectorySetpointCallback(SetPosition, SetVelocity, SetYaw)

    # Set Attitude
    def SetAttitude(self, SetQuaternion, BodyRate, SetThrust, SetYawRate):
        self.VehicleAttitudeSetpointCallback(SetQuaternion, BodyRate, SetThrust, SetYawRate)
    
    # Set Rate
    #def SetRate(self, SetRate, SetThrust):
    #    self.VehicleRatesSetpointCallback(SetRate, SetThrust)
        
    def Quaternion2Euler(self, w, x, y, z):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        Roll = math.atan2(t0, t1) * 57.2958

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Pitch = math.asin(t2) * 57.2958

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Yaw = math.atan2(t3, t4) * 57.2958

        return Roll, Pitch, Yaw
    
    # Euler to Quaternion
    def Euler2Quaternion(self, Roll, Pitch, Yaw):
        CosYaw = math.cos(Yaw * 0.5)
        SinYaw = math.sin(Yaw * 0.5)
        CosPitch = math.cos(Pitch * 0.5)
        SinPitch = math.sin(Pitch * 0.5)
        CosRoll = math.cos(Roll * 0.5)
        SinRoll= math.sin(Roll * 0.5)
        
        w = CosRoll * CosPitch * CosYaw + SinRoll * SinPitch * SinYaw
        x = SinRoll * CosPitch * CosYaw - CosRoll * SinPitch * SinYaw
        y = CosRoll * SinPitch * CosYaw + SinRoll * CosPitch * SinYaw
        z = CosRoll * CosPitch * SinYaw - SinRoll * CosPitch * CosYaw
        
        return w, x, y, z
    
    # VehicleAttitudeSetpoint
    def VehicleAttitudeSetpointCallback(self, SetQuaternion, BodyRate, SetThrust, SetYawRate):
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self.timestamp_offboard

        msg.roll_body = 0.0
        msg.pitch_body = 0.0
        msg.yaw_body = 0.0
        
        msg.q_d[0] = SetQuaternion[0]
        msg.q_d[1] = SetQuaternion[1]
        msg.q_d[2] = SetQuaternion[2]
        msg.q_d[3] = SetQuaternion[3]
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -SetThrust
        msg.yaw_sp_move_rate = SetYawRate
        
        self.VehicleAttitudeSetpointPublisher_.publish(msg)
        
    def TrajectorySetpointCallback(self, SetPosition, SetVelocity, SetYaw):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_offboard
        msg.x = SetPosition[0]
        msg.y = SetPosition[1]
        msg.z = SetPosition[2]
        msg.vx = SetVelocity[0]
        msg.vy = SetVelocity[1]
        msg.vz = SetVelocity[2]
        msg.yaw = SetYaw

        self.TrajectorySetpointPublisher_.publish(msg)
        
    def LidarCallback(self, msg):
        ObsDist = min(msg.ranges)
        ObsDist2 = max(msg.ranges)
        self.ObsPos[0] = ObsDist * math.cos(self.ObsAngle * math.pi / 180)
        self.ObsPos[1] = ObsDist * math.sin(self.ObsAngle * math.pi / 180)
        self.ObsAngle = 3.6 * np.argmin(msg.ranges)
        ObsSizeAngle = (3.6 * (100 - msg.ranges.count(math.inf))) / 2
        self.ObsSize = 2 * (ObsDist * math.tan(ObsSizeAngle * np.pi / 180))
        self.requestFlag = False
        # print(" min Distance : %f"%self.ObsDist)
        # print("Angle : %f"%self.ObsAngle)
        # print("max Distance : %f"%self.ObsDist2)
        #print("Position X : %f"%self.ObsPos[0])
        #print("Position Y : %f"%self.ObsPos[1])
        #print("X : %f"%self.ObsPos[0],"Y : %f"%self.ObsPos[1])
        # print(" ObsSize : %f"%self.ObsSize )
        # print(" ObsSizeAngle : %f"%self.ObsSizeAngle )
        if self.z <-4.0:
            if ObsDist < 2.5:
                self.requestFlag = True
