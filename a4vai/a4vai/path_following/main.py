
import rclpy
import threading

# from rclpy.executors import MultiThreadedExecutor

from a4vai.path_following.pf_attitude_cmd_module import PFAttitudeCmdModule
from a4vai.path_following.pf_gpr_module import PF_GPR_Module
from a4vai.path_following.pf_guid_param_module import PFGuidModule


def main(args=None):
    rclpy.init(args=args)
    
    pf_attitude_cmd_module = PFAttitudeCmdModule()
    # pf_gpr_module = PF_GPR_Module()
    pf_guid_module = PFGuidModule()

    try : 
        rclpy.spin(pf_attitude_cmd_module)
    except Exception as e:
                    pf_attitude_cmd_module.get_logger().info(
                        'MPPI module Start failed %r' % (e,))
    # try : 
    #     rclpy.spin(pf_gpr_module)
    # except Exception as e:
    #                 pf_gpr_module.get_logger().info(
    #                     'MPPI module Start failed %r' % (e,))
    try :       
        rclpy.spin(pf_guid_module)
    except Exception as e:
                    pf_guid_module.get_logger().info(
                        'MPPI module Start failed %r' % (e,))
    finally :
        # pf_attitude_cmd_module.destroy_node()
        # pf_gpr_module.destroy_node()
        # pf_guid_module.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
