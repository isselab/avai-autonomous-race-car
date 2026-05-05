import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from racecar_msgs.action import Server

import time

class DrivingNode(Node):
    def __init__(self):
       super().__init__('driving_node')
       self.connected = False
       self.is_driving = False
       self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
       self.start_sub = self.create_subscription(String, "control_driving", self.start_navigation, 10)
       self.action_server = ActionServer(
            self,
            Server,
            "simple_driving",
            self.action_callback
        )

    def start_navigation(self, input_msg):
        if input_msg.data == "start":
            self.is_driving = True
            msg = Twist()
            msg.linear.x = .1
            self.publisher.publish(msg)

        elif input_msg.data == "stop":
            self.is_driving = False
            msg = Twist()
            msg.linear.x = 0.0
            self.publisher.publish(msg)

    def action_callback(self, goal_handle):
        if goal_handle.request.connect == "connect":
            self.connected = True
            feedback_msg = Server.Feedback()
            feedback_msg.is_driving = "Connection Established"
            goal_handle.publish_feedback(feedback_msg)

            while self.connected:
                driving_or_stopped = "driving" if self.is_driving else "stopped"
                feedback_msg = Server.Feedback()
                feedback_msg.is_driving = driving_or_stopped
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)

        else:
            print("Could not authenticate. Disconnecting...")
        result = Server.Result()
        result.disconnect = "disconnected"
        return result


def main(args=None):
    rclpy.init(args=args)

    driving_node = DrivingNode()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(driving_node)

    try:
        executor.spin()
    finally:
        driving_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
