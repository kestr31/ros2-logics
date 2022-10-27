
import rclpy


# from rclpy.executors import MultiThreadedExecutor

from a4vai.collision_avoidance.CollisionAvoidance import Collision_Avoidance

def main(args=None):
    rclpy.init(args=args)
    collision_avoidance_module = Collision_Avoidance()
    
    try:
        rclpy.spin(collision_avoidance_module)
        
    except KeyboardInterrupt:
        collision_avoidance_module.get_logger().info('Keyboard Interrupt (SIGINT)')
       
    finally:
        collision_avoidance_module.destroy_node()
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()
