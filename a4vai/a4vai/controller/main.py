
import rclpy


# from rclpy.executors import MultiThreadedExecutor

from a4vai.controller.controller import ControllerNode
from a4vai.controller.path_plan_service import PathPlanningService


def main(args=None):
    rclpy.init(args=args)
    Controller = ControllerNode()    
    try:
        rclpy.spin(Controller)
    except KeyboardInterrupt:
        Controller.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        Controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# executor = rclpy.executors.MultiThreadedExecutor()
# executor.add_node(node)
# executor.add_node(node2)
# # Spin in a separate thread
# executor_thread = threading.Thread(target=executor.spin, daemon=True)
# executor_thread.start()
# rate = node.create_rate(2)
# try:
#     while rclpy.ok():
#         print('Help me body, you are my only hope')
#         rate.sleep()
# except KeyboardInterrupt:
#     pass
# rclpy.shutdown()
# executor_thread.join()