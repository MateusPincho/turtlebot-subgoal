import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from tf2_ros import Buffer, TransformListener

class FuzzyPlanner(Node):
    def __init__(self):
        super().__init__('fuzzy_planner_node')

        # Subscribers
        self.rviz_goal_pose = self.create_subscription(PoseStamped,"/move_base_simple/goal",self.goal_callback,10)      # Subscribes to /move_base_simple/goal when using the RVIZ node with cartographer
        #self.lidar_distance_sub = self.create_subscription(Float32MultiArray,"/lidar_sectors",self.distance_callback,10)

        # Publishers
        self.subgoals_pub = self.create_publisher(PoseStamped,"/subgoals",10)

        # The buffer stores transform data for up to 10 seconds by default
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot final goal pose
        self.goal_pose = None

        self.get_logger.info("Initializing the Fuzzy Planner Node. Waiting for goal position...")


    def get_robot_pose(self):
        '''
        Return the robot pose (x,y,theta) extracted from /map to /base_link
        '''

        # Get the latest available transform from /map to /base_link    
        t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())

        # Extract Position
        x = t.transform.translation.x
        y = t.transform.translation.y
        theta = self.get_yaw_from_quaternion(t.transform.rotation)

        return x, y, theta

    def goal_callback(self, msg:PoseStamped):

        # Rebecer goal final do RViZ
        self.get_logger().info("New goal received! Planning...")
        self.goal_pose = msg.pose

        x,y,t = self.get_robot_pose()
        self.get_logger().info(f"Robot position is ({x},{y})")
        

    #def distance_callback(self, msg:Float32MultiArray):
        # Recebe as distancias aos obstaculos e gera os novos subgoals a partir da logica fuzzy

    @staticmethod
    def get_yaw_from_quaternion(q):
        import math
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = FuzzyPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()