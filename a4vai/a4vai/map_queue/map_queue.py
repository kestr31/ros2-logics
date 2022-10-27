import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import math


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


# from model_spawn.model_spawn_srvs.srv import MakeWorld
from model_spawn_srvs.srv import MapGeneration
from msg_srv_act_interface.srv import MapInit

class MapQueueNode(Node):
    def __init__(self):
        super().__init__('map_queue')
        self.get_logger().info('running...')
        self.mapgenflag = False
        self.MapSequenceService_ = self.create_service(MapInit, 'map_sequence', self.MapSequenceCallback)
        self.Period = 1/10
        # self.Timer = self.create_timer(self.Period, self.TimerCallback)


    def MapSequenceCallback(self, request, response):
        self.mapgenflag = request.request_init_map
        if self.mapgenflag is True : 
            print("===== Start Map Generation =====")
            print(self.mapgenflag)
            self.TimerCallback()
            response.map_sequence_init = True
            return response
        else : 
            response.map_sequence_init = False
            print("===== Wait for Map Generation Flag =====")
            return response
            
            
    def TimerCallback(self):
        if self.mapgenflag is True : 
            map_gen = MapGenerationNode()
            map_gen.RequestMapGeneration()
            rclpy.spin_once(map_gen)
            if map_gen.futures.done():
                try : 
                    map_gen.result = map_gen.futures.result()
                except Exception as e:
                    map_gen.get_logger().info(
                        'Service call failed %r' % (e,))
                else :
                    map_gen.get_logger().info("Map Generation ")
                    if map_gen.result.map_generation is True :
                        self.mapgenflag = False
                        map_gen.destroy_node()
                    
            else :
                pass
        else : 
            self.get_logger().info('Waiting for Request !!')

        
class MapGenerationNode(Node):
    def __init__(self):
        super().__init__("map_gen")
        self.declare_service_client_custom()
        self.result = False
        
    def RequestMapGeneration(self):
        self.map_generation_request = MapGeneration.Request()
        self.map_generation_request.world_generation_request = True
        self.futures = self.MapGenerationServiceClient_.call_async(self.map_generation_request)
        # rclpy.spin_until_future_complete(self, futures)
        # if self.result.world_generation_request is True : 
        #     print("Map Generation Complete!!")
            
    def declare_service_client_custom(self):
        self.MapGenerationServiceClient_ = self.create_client(MapGeneration, 'map_generation')
        while not self.MapGenerationServiceClient_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map Generation service not available, waiting again...')
        print("====== custom Service Open ======")
        
        
def main(args=None):
    rclpy.init(args=args)
    map_queue = MapQueueNode()
    print("gen Node")
    
    try:
        rclpy.spin(map_queue)
    except KeyboardInterrupt:
        map_queue.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        map_queue.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()