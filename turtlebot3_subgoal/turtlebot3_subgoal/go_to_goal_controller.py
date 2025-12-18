import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data, QoSProfile

class GoToGoal(Node):
    def __init__(self):
        super().__init__('fuzzy_planner_node')


        self.subgoals_sub = self.create_subscription(
            PoseStamped,
            "/subgoals",
            self.subgoal_callback,
            10
        )

    def subgoal_callback(self, msg:PoseStamped):

        # Rebecer subgoal do callback e publicar o cmd_vel usando o controlador Lyapunov
