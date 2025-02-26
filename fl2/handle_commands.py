import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger

# Import the RealSense or Vicon nodes
from .realsense_sys_node import RealSense
from .vicon_sys_node import Vicon

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=1)

class CommNode(Node):
    def __init__(self, running_node):
        super().__init__('rob498_drone_1')
        print('node init')
        self.running_node = running_node  # Store the RealSense or Vicon instance
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_1/comm/launch', self.callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_1/comm/test', self.callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_1/comm/land', self.callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_1/comm/abort', self.callback_abort, qos_profile=qos_profile)
        print('services created')

        self.set_init = True
        self.height = 0.3

    def callback_launch(self, request, response):
        print('Launch Requested. Your drone should take off.')
        if self.set_init:
            self.running_node.set_pose_initial()
            self.set_init = False
        self.running_node.set_position.z = self.height
        return response

    def callback_test(self, request, response):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        return response

    def callback_land(self, request, response):
        print('Land Requested. Your drone should land.')
        self.running_node.set_position.z = 0.0
        self.set_init = True
        return response

    def callback_abort(self, request, response):
        print('Abort Requested. Your drone should land immediately due to safety considerations.')
        response.success = True
        response.message = "Success"
        self.running_node.set_position.z = 0.0
        return response

def main(args=None):
    rclpy.init(args=args) 
    print('starting node')
    
    # running_node = RealSense()
    running_node = Vicon()
    comm_node = CommNode(running_node)  # Pass RealSense instance
    # rclpy.spin(running_node)  # Start RealSense node
    rclpy.spin(comm_node)  # Start CommNode

    running_node.destroy_node()
    comm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Empty, Trigger

# # import the RealSense or Vicon nodes
# from .realsense_sys_node import RealSense
# from .vicon_sys_node import Vicon

# # Setup the node
# running_node = RealSense()
# # running_node = Vicon()

# # Callback handlers
# def handle_launch():
#     print('Launch Requested. Your drone should take off.')
#     # Set the position to current pose but 1.5 m higher
#     running_node.set_position = running_node.position
#     running_node.set_orientation = running_node.orientation
#     running_node.set_position.z = running_node.init_position.z + 1.5

# def handle_test():
#     print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
#     running_node.set_position = running_node.position
#     running_node.set_orientation = running_node.orientation

# def handle_land():
#     print('Land Requested. Your drone should land.')
#     # Set the position where it started
#     running_node.set_position = running_node.init_position
#     running_node.set_orientation = running_node.init_orientation

# def handle_abort():
#     print('Abort Requested. Your drone should land immediately due to safety considerations')
#     # Set the position where it started
#     running_node.set_position = running_node.init_position
#     running_node.set_orientation = running_node.init_orientation

# # Service callbacks
# def callback_launch(request, response):
#     handle_launch()
#     return response

# def callback_test(request, response):
#     handle_test()
#     return response

# def callback_land(request, response):
#     handle_land()
#     return response

# def callback_abort(request, response):
#     handle_abort()
#     return response

# class CommNode(Node):
#     def __init__(self):
#         super().__init__('rob498_drone_XX')
#         self.srv_launch = self.create_service(Trigger, 'rob498_drone_XX/comm/launch', callback_launch)
#         self.srv_test = self.create_service(Trigger, 'rob498_drone_XX/comm/test', callback_test)
#         self.srv_land = self.create_service(Trigger, 'rob498_drone_XX/comm/land', callback_land)
#         self.srv_abort = self.create_service(Trigger, 'rob498_drone_XX/comm/abort', callback_abort)


# def main(args=None):
#     rclpy.init(args=args)
#     comm_node = CommNode()
#     rclpy.spin(running_node)
#     initial_position()
#     rclpy.spin(comm_node)
#     running_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = CommNode()
# #     rclpy.spin(node)
# #     rclpy.shutdown()

# # if __name__ == "__main__":
# #     main()


# import rclpy
# import time
# from rclpy.node import Node
# from .realsense_sys_node import RealSense

# class DroneMission(Node):
#     def __init__(self, running_node):
#         super().__init__('drone_mission')
#         self.running_node = running_node  # Store RealSense instance
#         self.set_init = False
    
#     def initial_pose(self):
#         self.running_node.init_position = self.running_node.position
#         self.running_node.init_orientation = self.running_node.orientation
#         self.set_init = True
    
#     def launch(self):
#         self.get_logger().info('Launch: Taking off...')
#         if not self.set_init:
#             self.initial_pose()
#         self.running_node.set_position = self.running_node.position
#         self.running_node.set_orientation = self.running_node.orientation
#         self.running_node.set_position.z = self.running_node.init_position.z + 1.5
#         self.running_node.set_vision_to_setpoint = False
    
#     def test(self):
#         self.get_logger().info('Test: Performing tasks...')
#         self.running_node.set_position = self.running_node.position
#         self.running_node.set_orientation = self.running_node.orientation
    
#     def land(self):
#         self.get_logger().info('Land: Returning to ground...')
#         self.running_node.set_position = self.running_node.init_position
#         self.running_node.set_orientation = self.running_node.init_orientation
#         self.running_node.set_vision_to_setpoint = False
    
#     def execute_mission(self):
#         self.get_logger().info("Mission Starting...")
#         time.sleep(5)
#         self.launch()
        
#         time.sleep(8)
#         self.test()
        
#         time.sleep(10)
#         self.land()
        
#         self.get_logger().info("Mission Completed!")

# def main(args=None):
#     rclpy.init(args=args)
#     running_node = RealSense()
#     mission = DroneMission(running_node)
    
#     rclpy.spin_once(mission, timeout_sec=0.1)  # Ensure ROS starts properly
#     mission.execute_mission()
    
#     running_node.destroy_node()
#     mission.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()








###############
# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Empty, Trigger

# # Import the RealSense or Vicon nodes
# from .realsense_sys_node import RealSense
# from .vicon_sys_node import Vicon

# class CommNode(Node):
#     def __init__(self, running_node):
#         super().__init__('rob498_drone_1')
#         self.running_node = running_node  # Store the RealSense or Vicon instance
#         self.srv_launch = self.create_service(Trigger, 'rob498_drone_1/comm/launch', self.callback_launch)
#         self.srv_test = self.create_service(Trigger, 'rob498_drone_1/comm/test', self.callback_test)
#         self.srv_land = self.create_service(Trigger, 'rob498_drone_1/comm/land', self.callback_land)
#         self.srv_abort = self.create_service(Trigger, 'rob498_drone_1/comm/abort', self.callback_abort)

#         self.set_init = True

#     def callback_launch(self, request, response):
#         print('Launch Requested. Your drone should take off.')
#         if self.set_init:
#             self.running_node.set_pose_initial()
#             self.set_init = False
#         # self.running_node.set_position = self.running_node.position
#         # self.running_node.set_orientation = self.running_node.orientation
#         self.running_node.set_position.z = 1.5
#         return response

#     def callback_test(self, request, response):
#         print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
#         # self.running_node.set_position = self.running_node.position
#         # self.running_node.set_orientation = self.running_node.orientation
#         return response

#     def callback_land(self, request, response):
#         print('Land Requested. Your drone should land.')
#         # self.running_node.set_position = self.running_node.init_position
#         # self.running_node.set_orientation = self.running_node.init_orientation
#         self.running_node.set_position.z = 0
#         self.set_init = True
#         return response

#     def callback_abort(self, request, response):
#         print('Abort Requested. Your drone should land immediately due to safety considerations.')
#         # self.running_node.set_position.z = self.running_node.init_position.z
#         self.running_node.set_position.z = 0
#         return response

# def main(args=None):
#     rclpy.init(args=args) 
    
#     running_node = RealSense()
#     comm_node = CommNode(running_node)  # Pass RealSense instance
#     rclpy.spin(running_node)  # Start RealSense node
#     rclpy.spin(comm_node)  # Start CommNode

#     running_node.destroy_node()
#     comm_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()