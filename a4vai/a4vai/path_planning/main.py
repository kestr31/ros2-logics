
import rclpy


# from rclpy.executors import MultiThreadedExecutor

from a4vai.path_planning.deep_sac_module import DeepSACNode


def main(args=None):
    rclpy.init(args=args)
    SAC_module = DeepSACNode()
    try:
        rclpy.spin(SAC_module)
    except KeyboardInterrupt:
        SAC_module.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        SAC_module.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
