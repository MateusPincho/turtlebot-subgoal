import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from tf2_ros import Buffer, TransformListener

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')

        # Subscriber
        #self.subgoals_sub = self.create_subscription(PoseStamped,"/subgoals",self.subgoal_callback,10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # The buffer stores transform data for up to 10 seconds by default
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
              

    def get_robot_pose(self):

        # Get the latest available transform from /map to /base_link    
        t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())

        # Extract Position
        x = t.transform.translation.x
        y = t.transform.translation.y
        theta = get_yaw_from_quaternion(t.transform.rotatation)
        
        return x, y, theta

    # def subgoal_callback(self, msg:PoseStamped):

    #     # Rebecer subgoal do callback e publicar o cmd_vel usando o controlador Lyapunov

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