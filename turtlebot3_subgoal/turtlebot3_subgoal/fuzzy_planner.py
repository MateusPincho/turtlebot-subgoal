import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener

import math
import numpy as np

# Global variables for fuzzy logic
dist_fuzzy = {"N": 0, "M": 0, "F": 0}
alpha_fuzzy = {"WE": 0, "NW": 0, "NO": 0, "NE": 0, "ES": 0}
dist_fuzzy_tracking = {"N": 0, "M": 0, "F": 0}
alpha_fuzzy_tracking = {"WE": 0, "NW": 0, "NO": 0, "NE": 0, "ES": 0}

# Variáveis para armazenar os últimos inputs fuzzy calculados (acessadas na atuação)
dNW_val = {}
dNO_val = {}
dNE_val = {}
dES_val = {}
dWE_val = {}


def muNear(x):
    min_dist = 400
    max_dist = 600
    if x <= min_dist:
        return 1.0
    if min_dist < x < max_dist:
        return (max_dist - x) / (max_dist - min_dist)
    return 0.0


def muMedium(x):
    min_dist = 400
    cen_dist = 600
    max_dist = 800
    if x <= min_dist or x >= max_dist:
        return 0.0
    if min_dist < x <= cen_dist:
        return (x - min_dist) / (cen_dist - min_dist)
    if cen_dist < x < max_dist:
        return (max_dist - x) / (max_dist - cen_dist)
    return 0.0


def muFar(x):
    min_dist = 800
    max_dist = 1000
    if x <= min_dist:
        return 0.0
    if min_dist < x < max_dist:
        return (x - min_dist) / (max_dist - min_dist)
    return 1.0

def get_fuzzy_inputs(distances):  # In meters
    global dNW_val, dNO_val, dNE_val, dES_val, dWE_val

    # Convert to meters and limit result to 1 meter
    dNW, dNO, dNE, dES, dWE = [d * 1000 if d * 1000 < 1000 else 1000 for d in distances]

    dNW_val = {"N": muNear(dNW), "M": muMedium(dNW), "F": muFar(dNW)}
    dNO_val = {"N": muNear(dNO), "M": muMedium(dNO), "F": muFar(dNO)}
    dNE_val = {"N": muNear(dNE), "M": muMedium(dNE), "F": muFar(dNE)}
    dES_val = {"N": muNear(dES), "M": muMedium(dES), "F": muFar(dES)}
    dWE_val = {"N": muNear(dWE), "M": muMedium(dWE), "F": muFar(dWE)}

    return fuzzy_inference()


def fuzzy_rules_logic(rule, label_dist, label_alpha):
    dist_fuzzy[label_dist] += rule
    alpha_fuzzy[label_alpha] += rule


def fuzzy_rules_tracking_logic(regra, label_dist, label_alpha):
    dist_fuzzy_tracking[label_dist] += regra
    alpha_fuzzy_tracking[label_alpha] += regra


def fuzzy_inference():
    global dist_fuzzy, alpha_fuzzy, dist_fuzzy_tracking, alpha_fuzzy_tracking

    # Reset accumulators
    dist_fuzzy = {k: 0 for k in ["N", "M", "F"]}
    alpha_fuzzy = {k: 0 for k in ["WE", "NW", "NO", "NE", "ES"]}
    dist_fuzzy_tracking = {k: 0 for k in ["N", "M", "F"]}
    alpha_fuzzy_tracking = {k: 0 for k in ["WE", "NW", "NO", "NE", "ES"]}

    rotation = {"NW": 0, "NE": 0}

    # Rotation logic
    for label in ["N", "M", "F"]:
        if dWE_val[label] <= dES_val[label]:
            rotation["NW"] += 1
        else:
            rotation["NE"] += 1

    rotation_direction = "NW" if rotation["NW"] >= rotation["NE"] else "NE"

    # Rules
    fuzzy_rules_logic(
        min(dNW_val["N"], dNO_val["N"], dNE_val["N"]), "N", rotation_direction
    )
    fuzzy_rules_logic(min(dNW_val["N"], dNO_val["N"], dNE_val["M"]), "N", "NE")
    fuzzy_rules_logic(min(dNW_val["N"], dNO_val["N"], dNE_val["F"]), "N", "NE")
    fuzzy_rules_logic(
        min(dNW_val["N"], dNO_val["M"], dNE_val["N"]), "N", rotation_direction
    )
    fuzzy_rules_logic(min(dNW_val["N"], dNO_val["M"], dNE_val["M"]), "M", "NE")
    fuzzy_rules_logic(min(dNW_val["N"], dNO_val["M"], dNE_val["F"]), "M", "NE")
    fuzzy_rules_logic(min(dNW_val["N"], dNO_val["F"], dNE_val["N"]), "M", "NO")
    fuzzy_rules_logic(min(dNW_val["N"], dNO_val["F"], dNE_val["M"]), "M", "NE")
    fuzzy_rules_logic(min(dNW_val["N"], dNO_val["F"], dNE_val["F"]), "F", "NE")
    fuzzy_rules_logic(min(dNW_val["M"], dNO_val["N"], dNE_val["N"]), "N", "NW")
    fuzzy_rules_logic(
        min(dNW_val["M"], dNO_val["N"], dNE_val["M"]), "N", rotation_direction
    )
    fuzzy_rules_logic(min(dNW_val["M"], dNO_val["N"], dNE_val["F"]), "N", "NE")
    fuzzy_rules_logic(min(dNW_val["M"], dNO_val["M"], dNE_val["N"]), "M", "NW")
    fuzzy_rules_logic(
        min(dNW_val["M"], dNO_val["M"], dNE_val["M"]), "M", rotation_direction
    )
    fuzzy_rules_logic(min(dNW_val["M"], dNO_val["M"], dNE_val["F"]), "M", "NE")
    fuzzy_rules_logic(min(dNW_val["M"], dNO_val["F"], dNE_val["N"]), "M", "NW")
    fuzzy_rules_logic(min(dNW_val["M"], dNO_val["F"], dNE_val["M"]), "F", "NO")
    fuzzy_rules_logic(min(dNW_val["M"], dNO_val["F"], dNE_val["F"]), "F", "NE")
    fuzzy_rules_logic(min(dNW_val["F"], dNO_val["N"], dNE_val["N"]), "N", "NW")
    fuzzy_rules_logic(min(dNW_val["F"], dNO_val["N"], dNE_val["M"]), "N", "NW")
    fuzzy_rules_logic(
        min(dNW_val["F"], dNO_val["N"], dNE_val["F"]), "N", rotation_direction
    )
    fuzzy_rules_logic(min(dNW_val["F"], dNO_val["M"], dNE_val["N"]), "F", "NW")
    fuzzy_rules_logic(min(dNW_val["F"], dNO_val["M"], dNE_val["M"]), "M", "NW")
    fuzzy_rules_logic(
        min(dNW_val["F"], dNO_val["M"], dNE_val["F"]), "M", rotation_direction
    )
    fuzzy_rules_logic(min(dNW_val["F"], dNO_val["F"], dNE_val["N"]), "F", "NW")
    fuzzy_rules_logic(min(dNW_val["F"], dNO_val["F"], dNE_val["M"]), "F", "NW")
    fuzzy_rules_logic(min(dNW_val["F"], dNO_val["F"], dNE_val["F"]), "F", "NO")

    # Tracking
    fuzzy_rules_tracking_logic(min(dWE_val["N"], dES_val["N"]), "F", "NO")
    fuzzy_rules_tracking_logic(min(dWE_val["N"], dES_val["M"]), "M", "NE")
    fuzzy_rules_tracking_logic(min(dWE_val["N"], dES_val["F"]), "M", "NO")
    fuzzy_rules_tracking_logic(min(dWE_val["M"], dES_val["N"]), "M", "NW")
    fuzzy_rules_tracking_logic(min(dWE_val["M"], dES_val["M"]), "F", "NO")
    fuzzy_rules_tracking_logic(min(dWE_val["M"], dES_val["F"]), "M", "NW")
    fuzzy_rules_tracking_logic(min(dWE_val["F"], dES_val["N"]), "M", "NO")
    fuzzy_rules_tracking_logic(min(dWE_val["F"], dES_val["M"]), "M", "NE")
    fuzzy_rules_tracking_logic(min(dWE_val["F"], dES_val["F"]), "F", "NO")

    return deffuzification()


def deffuzification():
    e_cv = {"N": 0, "M": 2, "F": 3}

    # Sub Goal Distance Tracking
    num, den = 0, 0
    for k, cv in e_cv.items():
        w = dist_fuzzy.get(k, 0)
        num += w * cv
        den += w
    dist_sub_goal = num / den if den != 0 else 0

    # Sub Goal Distance Tracking
    num, den = 0, 0
    for k, cv in e_cv.items():
        w = dist_fuzzy_tracking.get(k, 0)
        num += w * cv
        den += w
    dist_sub_goal_tracking = num / den if den != 0 else 0

    alpha_cv = {
        "ES": -math.pi + 2 * math.pi / 4,
        "NE": -math.pi + 3 * math.pi / 4,
        "NO": -math.pi + 4 * math.pi / 4,
        "NW": -math.pi + 5 * math.pi / 4,
        "WE": -math.pi + 6 * math.pi / 4,
    }

    # Alpha Sub Goal
    num, den = 0, 0
    for k, cv in alpha_cv.items():
        w = alpha_fuzzy.get(k, 0)
        num += w * cv
        den += w
    alpha_sub_goal = num / den if den != 0 else 0

    if dist_sub_goal <= 0.01:
        dist_sub_goal = 0.01

    # Alpha Sub Goal Tracking
    num, den = 0, 0
    for k, cv in alpha_cv.items():
        w = alpha_fuzzy_tracking.get(k, 0)
        num += w * cv
        den += w
    alpha_sub_goal_tracking = num / den if den != 0 else 0

    if dist_sub_goal_tracking <= 1:
        dist_sub_goal_tracking = 1

    return [
        dist_sub_goal,
        alpha_sub_goal,
        dist_sub_goal_tracking,
        alpha_sub_goal_tracking,
    ]

def fuzzy_and_lyapunov(
    node, robot_position, robot_orientation, goal_position, goal_orientation, distances
):
    x, y = robot_position

    xg_perm, yg_perm = goal_position

    change = get_fuzzy_inputs(distances)
    theta = robot_orientation

    phi_perm = math.atan2((yg_perm - y), (xg_perm - x))
    Dfix = math.sqrt((yg_perm - y) ** 2 + (xg_perm - x) ** 2)

    xg, yg = xg_perm, yg_perm

    mode = "Not Defined"
    # State Logic (Obstacle Avoidance / Tracking / Goal Seeking)
    if dNW_val["F"] < 1 or dNO_val["F"] < 1 or dNE_val["F"] < 1:
        if Dfix >= 0.3:
            e, alpha_fuzzy_val = change[0], change[1]
            xg = x + e * math.cos(alpha_fuzzy_val + theta)
            yg = y + e * math.sin(alpha_fuzzy_val + theta)
            mode = "Obstacle Avoidance"

    elif (dES_val["N"] > 0 or dWE_val["N"] > 0) and (
        math.pi / 4 <= abs(phi_perm) - abs(theta) <= 3 * math.pi / 4
    ):
        if Dfix >= 1:
            e, alpha_fuzzy_val = change[2], change[3]
            xg = x + e * math.cos(alpha_fuzzy_val + theta)
            yg = y + e * math.sin(alpha_fuzzy_val + theta)
            mode = "Tracking"
    else:
        mode = "Goal Seeking"

    node.get_logger().info(f"{mode} Mode")
    marker = node.create_marker(xg, yg, mode)
    node.marker_pub.publish(marker)

    # Kinematic control
    dist_x, dist_y = xg - x, yg - y
    D = math.sqrt(dist_x**2 + dist_y**2)
    phi = math.atan2(dist_y, dist_x)
    alpha = theta - phi
    alpha = math.atan2(math.sin(alpha), math.cos(alpha)) 

    node.get_logger().info(f"Error distance: {D:.2f} | Orientation Error: {alpha:.2f}")

    if Dfix < 0.1:
        return 0.0, 0.0
    
    v_max = 0.25
    w_max = 1.80
    K_w, K_v = 2.07/4, 1.49/4 

    v = K_v * D * math.cos(alpha)
    v = np.clip(v, -v_max, v_max)

    w = -K_w * alpha - (v / D) * math.sin(alpha)
    w = np.clip(w, -w_max, w_max)

    return v, w

def lyapunov_control(
    node, robot_position, robot_orientation, goal_position
):
    x, y = robot_position
    xg, yg = goal_position
    theta = robot_orientation

    dist_x, dist_y = xg - x, yg - y
    D = math.sqrt(dist_x**2 + dist_y**2)
    phi = math.atan2(dist_y, dist_x)
    alpha = theta - phi
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))  

    node.get_logger().info(f"Error distance: {D:.2f} | Orientation Error: {alpha:.2f}")

    if D < 0.1:
        return 0.0, 0.0
    
    v_max = 0.25
    w_max = 1.80
    K_w, K_v = 2.07/4, 1.49/4 

    v = K_v * D * math.cos(alpha)
    v = np.clip(v, -v_max, v_max)

    w = -K_w * alpha - (v / D) * math.sin(alpha)
    w = np.clip(w, -w_max, w_max)

    return v, w


class FuzzyPlanner(Node):
    def __init__(self):
        super().__init__("fuzzy_planner_node")

        # Subscribers
        self.rviz_goal_pose = self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self.goal_callback, 10
        )  # Subscribes to /move_base_simple/goal when using the RVIZ node with cartographer
        self.lidar_distance_sub = self.create_subscription(
            Float32MultiArray, "/lidar_sectors", self.distance_callback, 10
        )

        # Publishers
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(Marker, "/fuzzy_subgoal", 10)

        # The buffer stores transform data for up to 10 seconds by default
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables
        self.position_r = None
        self.orientation_r = None
        self.position_g = None
        self.orientation_g = None

        self.get_logger().info(
            "Initializing the Fuzzy Planner Node. Waiting for goal position..."
        )

    def goal_callback(self, msg: PoseStamped):
        # Rebecer goal final do RViZ
        self.get_logger().info("New goal received! Planning...")
        self.goal_pose = msg.pose

        self.position_g = [
            self.goal_pose.position.x,
            self.goal_pose.position.y,
        ]
        self.orientation_g = self.get_yaw_from_quaternion(self.goal_pose.orientation)

    def distance_callback(self, msg: Float32MultiArray):
        self.get_logger().info("Received distances from lidar...")
        
        distances = list(msg.data)[:5]

        # Get the latest available transform from /map to /base_link
        try:
            # Try to look up the transform
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # If TF is not ready yet, simply return and wait for the next Lidar message
            self.get_logger().info(f'Transform not ready yet: {e}')
            return

        self.get_logger().info("Received tf...")

        # Extract Position
        self.position_r = [t.transform.translation.x, t.transform.translation.y]
        self.orientation_r = self.get_yaw_from_quaternion(t.transform.rotation)

        self.get_logger().info(f" Robot {self.position_r} | {self.orientation_r}...")

        if not None in [
            self.position_r,
            self.orientation_r,
            self.position_g,
            self.orientation_g,
        ]:
            v, w = fuzzy_and_lyapunov(
                self,
                self.position_r,
                self.orientation_r,
                self.position_g,
                self.orientation_g,
                distances,
            )

            #v, w = lyapunov_control(
            #    self,
            #    self.position_r,
            #    self.orientation_r,
            #    self.position_g,
            #)

            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w

            self.twist_pub.publish(msg)

            self.get_logger().info(
                f"Linear Velocity: {v:.2f} | Angular Velocity: {w:.2f}"
            )

            return 

        self.get_logger().info("Not enough info...")

    def create_marker(self, xg, yg, mode):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the scale (size in meters)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color (RGBA)
        marker.color.a = 1.0 # Opaque
        if mode == "Obstacle Avoidance":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        elif mode == "Tracking":
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

        elif mode == "Goal Seeking":
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        else:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        marker.pose.position.x = xg
        marker.pose.position.y = yg
        marker.pose.position.z = 0.0

        return marker

    @staticmethod
    def get_yaw_from_quaternion(q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
