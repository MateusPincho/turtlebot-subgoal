import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data, QoSProfile

class FuzzyPlanner(Node):
    def __init__(self):
        super().__init__('fuzzy_planner_node')

        self.rviz_goal_pose = self.create_subscription(
            PoseStamped, 
            "/goal_pose", 
            self.goal_callback, 
            10
        )

        self.lidar_distance_sub = self.create_subscription(
            Float32MultiArray,
            "/lidar_sectors",
            self.distance_callback,
            10
        )

        self.subgoals_pub = self.create_publisher(
            PoseStamped,
            "/subgoals",
            10
        )

    def goal_callback(self, msg:PoseStamped):

        # Rebecer goal final do RViZ
        return

    def distance_callback(self, msg:Float32MultiArray):

        # Recebe as distancias aos obstaculos e gera os novos subgoals a partir da logica fuzzy