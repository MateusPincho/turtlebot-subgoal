import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import math

class Aicardi(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        # Robot Parameters considering Turtlebot3
        self.declare_parameter('max_lin_velocity', 0.26)
        self.declare_parameter('max_ang_velocity', 1.82)
        self.declare_parameter('aicardi_h',1)
        self.declare_parameter('aicardi_gamma',1)
        self.declare_parameter('aicardi_k',1)

        self.max_lin_velocity = self.get_parameter('max_lin_velocity').get_parameter_value().double_value
        self.max_ang_velocity = self.get_parameter('max_ang_velocity').get_parameter_value().double_value
        self.aicardi_h = self.get_parameter('aicardi_h').get_parameter_value().double_value
        self.aicardi_gamma = self.get_parameter('aicardi_gamma').get_parameter_value().double_value
        self.aicardi_k = self.get_parameter('aicardi_k').get_parameter_value().double_value

        # Subscribers
        self.target_pose = None
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.create_timer(0.05, self.control_loop)

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

    def target_pose_callback(self, msg):
        self.target_pose = msg

    def control_loop(self):
        if self.robot_pose is None or self.target_pose is None:
            return

        # Robot pose
        x,y,theta = self.get_robot_pose()
        robot_pos = np.array([x, y])
        robot_ori = self.get_yaw_from_quaternion(self.robot_pose.pose.orientation)

        # Target pose
        target_pos = np.array([self.target_pose.pose.position.x, self.target_pose.pose.position.y])
        target_ori = self.get_yaw_from_quaternion(self.target_pose.pose.orientation)

        # Transform to Aicardi States
        dx, dy = target_pos - robot_pos
        target_distance = math.sqrt(dx**2 + dy**2)
        alpha = self.normalize_angle(np.arctan2(dy,dx) - robot_ori)
        theta_aicardi = self.normalize_angle(target_ori - np.arctan2(dy,dx))

        # Stopping condition
        if target_distance < 0.05:
            self.publish_velocity(0.0, 0.0)
            return

        # Calculate linear and angular velocities from (6) and (9)
        # u = gamma * cos(alpha) * e
        v = self.aicardi_gamma * math.cos(alpha) * target_distance
        
        # omega = k*alpha + gamma * (cos(alpha)*sin(alpha)/alpha) * (alpha + h*theta)
        omega = self.aicardi_k * alpha + self.aicardi_gamma * ((math.cos(alpha) * math.sin(alpha))/alpha) * (alpha +  self.aicardi_h * theta_aicardi)

        # Saturation Limits
        v = max(min(v, self.max_lin_velocity), -self.max_lin_velocity)
        omega = max(min(omega, self.max_ang_velocity), -self.max_ang_velocity)

        self.publish_velocity(v, omega)

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)

    @staticmethod
    def get_yaw_from_quaternion(q):
        import math
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def normalize_angle(angle):
        # Normalize angle to the range [-pi,pi)
        import numpy as np
        return np.mod(angle+np.pi, 2*np.pi) - np.pi

def main(args=None):
    rclpy.init(args=args)
    node = Aicardi()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
