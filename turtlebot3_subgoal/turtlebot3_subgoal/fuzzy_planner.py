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

class FuzzySystem:
    def __init__(self):
        self.variables = {}
        self.rules = []

    def add_variable(self, name, range_min, range_max, resolution=1000):
        self.variables[name] = {
            "range": np.linspace(range_min, range_max, resolution),
            "terms": {}
        }

    def add_term(self, var_name, term_name, mf_type, params, saturate=None):
        universe = self.variables[var_name]["range"]
        
        if mf_type == "Triangular":
            a, b, c = params
            mf = np.maximum(0, np.minimum((universe - a) / (b - a + 1e-9), (c - universe) / (c - b + 1e-9)))
            peak_start, peak_end = b, b

        elif mf_type == "Trapezoidal":
            a, b, c, d = params
            term1 = (universe - a) / (b - a + 1e-9)
            term2 = (d - universe) / (d - c + 1e-9)
            mf = np.maximum(0, np.minimum(np.minimum(term1, 1), term2))
            peak_start, peak_end = b, c
            
        elif mf_type == "Gaussian":
            # params = [mean, sigma]
            mean, sigma = params
            # Standard Gaussian formula
            mf = np.exp(-0.5 * ((universe - mean) / (sigma + 1e-9))**2)
            peak_start, peak_end = mean, mean
            
        else:
            raise ValueError("Unsupported MF type. Use 'Triangular', 'Trapezoidal', or 'Gaussian'.")
        
        if saturate == "Left":
            # Force everything to the left of the peak start to 1.0
            mf = np.where(universe <= peak_start, np.maximum(mf, 1.0), mf)
        elif saturate == "Right":
            # Force everything to the right of the peak end to 1.0
            mf = np.where(universe >= peak_end, np.maximum(mf, 1.0), mf)
        else:
            pass
            
        self.variables[var_name]["terms"][term_name] = mf

    def _get_membership_value(self, var_name, term_name, value):
        universe = self.variables[var_name]["range"]
        mf = self.variables[var_name]["terms"][term_name]
        return np.interp(value, universe, mf)
    
    def add_rule(self, antecedents, consequents, operator='and'):
        """
        consequents: List of (var_name, term_name) OR a single (var_name, term_name)
        """
        # Ensure consequents is always a list for consistent processing
        if isinstance(consequents, tuple):
            consequents = [consequents]
            
        self.rules.append({
            'if': antecedents, 
            'then': consequents, 
            'op': operator.lower()
        })

    def _internal_inference(self, inputs):
        """Helper to perform the fuzzy logic and return aggregated MFs."""
        aggregated_outputs = {}
        
        for rule in self.rules:
            # 1. Calculate activation level
            degrees = [self._get_membership_value(var, term, inputs.get(var, 0)) for var, term in rule['if']]
            activation = min(degrees) if rule['op'] == 'and' else max(degrees)
            
            # 2. Apply activation to ALL consequents in this rule
            for out_var, out_term in rule['then']:
                out_mf = self.variables[out_var]['terms'][out_term]
                clipped_mf = np.minimum(activation, out_mf)
                
                if out_var not in aggregated_outputs:
                    aggregated_outputs[out_var] = np.zeros_like(out_mf)
                
                # Aggregate using MAX (S-norm)
                aggregated_outputs[out_var] = np.maximum(aggregated_outputs[out_var], clipped_mf)
                
        return aggregated_outputs

    def compute(self, inputs):
        """Standard computation returning crisp values."""
        aggregated = self._internal_inference(inputs)
        results = {}
        for var_name, final_mf in aggregated.items():
            universe = self.variables[var_name]['range']
            sum_mu = np.sum(final_mf)
            results[var_name] = np.sum(final_mf * universe) / sum_mu if sum_mu > 0 else np.mean(universe)
        return results

fuzzy_nav = FuzzySystem()

# Inputs
fuzzy_nav.add_variable("Goal Distance", 0.0, 5.0)
fuzzy_nav.add_term("Goal Distance", "Very Near", "Gaussian", [1.0, 0.5], saturate="Left")
fuzzy_nav.add_term("Goal Distance", "Near", "Gaussian", [2.0, 0.5])
fuzzy_nav.add_term("Goal Distance", "Far", "Gaussian", [3.0, 0.5], saturate="Right")

fuzzy_nav.add_variable("Goal Orientation", -np.pi, np.pi)
fuzzy_nav.add_term("Goal Orientation", "Right", "Gaussian", [-np.pi / 2, 0.5], saturate="Left")
fuzzy_nav.add_term("Goal Orientation", "Center", "Gaussian", [0.0, 0.5])
fuzzy_nav.add_term("Goal Orientation", "Left", "Gaussian", [np.pi / 2, 0.5], saturate="Right")

wind_rose_keys = [
    "SW Distance",
    "WE Distance",
    "NW Distance",
    "NO Distance",
    "NE Distance",
    "ES Distance",
    "SE Distance",
    "SO Distance",
]
for k in wind_rose_keys:
    fuzzy_nav.add_variable(k, 0.0, 5.0)
    fuzzy_nav.add_term(k, "Very Near", "Gaussian", [0.4, 0.15], saturate="Left")
    fuzzy_nav.add_term(k, "Near", "Gaussian", [0.8, 0.15])
    fuzzy_nav.add_term(k, "Far", "Gaussian", [1.2, 0.15], saturate="Right")

# Outputs
fuzzy_nav.add_variable("D*", 0.0, 1.0)
fuzzy_nav.add_term("D*", "Very Near", "Gaussian", [0.0, 0.15])
fuzzy_nav.add_term("D*", "Near", "Gaussian", [0.4, 0.15])
fuzzy_nav.add_term("D*", "Far", "Gaussian", [0.8, 0.15], saturate="Right")

fuzzy_nav.add_variable("Alpha*", -np.pi, np.pi)
fuzzy_nav.add_term("Alpha*", "SOe", "Gaussian", [np.pi, 0.5])
fuzzy_nav.add_term("Alpha*", "SE", "Gaussian", [3.0 * np.pi / 4.0, 0.5])
fuzzy_nav.add_term("Alpha*", "ES", "Gaussian", [np.pi / 2.0, 0.5])
fuzzy_nav.add_term("Alpha*", "NE", "Gaussian", [np.pi / 4.0, 0.5])
fuzzy_nav.add_term("Alpha*", "NO", "Gaussian", [0.0, 0.5])
fuzzy_nav.add_term("Alpha*", "NW", "Gaussian", [-np.pi / 4.0, 0.5])
fuzzy_nav.add_term("Alpha*", "WE", "Gaussian", [-np.pi / 2.0, 0.5])
fuzzy_nav.add_term("Alpha*", "SW", "Gaussian", [-3.0 * np.pi / 4.0, 0.5])
fuzzy_nav.add_term("Alpha*", "SOw", "Gaussian", [-np.pi, 0.5])

fuzzy_nav.add_variable("Subgoal Confidence", 0.0, 1.0)
fuzzy_nav.add_term("Subgoal Confidence", "Low", "Gaussian", [0.0, 0.2])
fuzzy_nav.add_term("Subgoal Confidence", "Medium", "Gaussian", [0.5, 0.2])
fuzzy_nav.add_term("Subgoal Confidence", "High", "Gaussian", [1.0, 0.2])

# If everything is far away, explore terrain
fuzzy_nav.add_rule(
    [("NO Distance", "Far"), ("WE Distance", "Far"), ("ES Distance", "Far")], 
    [("D*", "Far"), ("Alpha*", "NO")], 
    operator="and"
)

# If there isn't anything close to a direction, go full speed at it
fuzzy_nav.add_rule(
    [("NO Distance", "Far")], 
    [("D*", "Far"), ("Alpha*", "NO")], 
    operator="or"
)

fuzzy_nav.add_rule(
    [("NW Distance", "Near")], 
    [("D*", "Near"), ("Alpha*", "NW")], 
    operator="or"
)

fuzzy_nav.add_rule(
    [("NE Distance", "Near")], 
    [("D*", "Near"), ("Alpha*", "NE")], 
    operator="or"
)

# Follow a narrow corridor carefuly
fuzzy_nav.add_rule(
    [("NO Distance", "Far"), ("WE Distance", "Very Near"), ("ES Distance", "Very Near")], 
    [("D*", "Near"), ("Alpha*", "NO")], 
    operator="and"
)

# Sprint through a wide corridor 
fuzzy_nav.add_rule(
    [("NO Distance", "Far"), ("WE Distance", "Near"), ("ES Distance", "Near")], 
    [("D*", "Near"), ("Alpha*", "NO")], 
    operator="and"
)

# Stay centered on corridor
fuzzy_nav.add_rule(
    [("ES Distance", "Very Near"), ("WE Distance", "Near"), ("NO Distance", "Far")], 
    [("D*", "Near"), ("Alpha*", "NW")], 
    operator="and"
)

fuzzy_nav.add_rule(
    [("ES Distance", "Near"), ("WE Distance", "Very Near"), ("NO Distance", "Far")], 
    [("D*", "Near"), ("Alpha*", "NE")], 
    operator="and"
)

# Turn on L-shape corridor
fuzzy_nav.add_rule(
    [("NO Distance", "Near"), ("WE Distance", "Very Near"), ("ES Distance", "Near")], 
    [("D*", "Very Near"), ("Alpha*", "NE")], 
    operator="and"
)

fuzzy_nav.add_rule(
    [("NO Distance", "Near"), ("WE Distance", "Near"), ("ES Distance", "Very Near")], 
    [("D*", "Very Near"), ("Alpha*", "NW")], 
    operator="and"
)

# Deadlock detected, turn around
fuzzy_nav.add_rule(
    [
        ("NO Distance", "Very Near"), 
        ("NE Distance", "Near"), 
        ("NW Distance", "Very Near"), 
        ("ES Distance", "Near"), 
        ("WE Distance", "Very Near")
    ], 
    [("D*", "Very Near"), ("Alpha*", "SOe")], 
    operator="and"
)

fuzzy_nav.add_rule(
    [
        ("NO Distance", "Very Near"), 
        ("NE Distance", "Very Near"), 
        ("NW Distance", "Near"), 
        ("ES Distance", "Very Near"), 
        ("WE Distance", "Near")
    ], 
    [("D*", "Very Near"), ("Alpha*", "SOw")], 
    operator="and"
)

def get_robot_goal_states(robot_position, robot_orientation, target_position):
    x, y = robot_position
    x_t, y_t = target_position
    D = ((x_t - x)**2 + (y_t - y)**2)**0.5
    phi = np.arctan2((y_t - y), (x_t - x))
    theta = robot_orientation
    alpha = theta - phi

    return D, alpha

def control_laws(D, Dd, alpha, Kv, Kw):
    ed = D - Dd
    v = Kv * ed * np.cos(alpha)
    omega = -Kw * alpha -v * np.sin(alpha) / D

    return v, omega

def wrap_to_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def alpha_to_wind_rose(alpha):
    alpha = wrap_to_pi(alpha)

    # Sector boundaries of 45°
    sector_edges = [
        (-np.pi/8,  np.pi/8,   "NO"),
        ( np.pi/8,  3*np.pi/8, "NE"),
        (3*np.pi/8, 5*np.pi/8, "ES"),
        (5*np.pi/8, 7*np.pi/8, "SE"),
        (7*np.pi/8, -7*np.pi/8,"SO"),  
        (-7*np.pi/8,-5*np.pi/8,"SW"),
        (-5*np.pi/8,-3*np.pi/8,"WE"),
        (-3*np.pi/8,-np.pi/8,  "NW")
    ]

    for low, high, label in sector_edges:
        if low < high:
            if low <= alpha < high:
                return label
        else:  # wrap-around case (South)
            if alpha >= low or alpha < high:
                return label

    # Fallback (should never happen)
    return "NO"

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
        self.robot_position = None
        self.robot_orientation = None
        self.goal_position = None
        self.goal_orientation = None

        self.get_logger().info(
            "Initializing the Fuzzy Planner Node. Waiting for goal position..."
        )

    def goal_callback(self, msg: PoseStamped):
        # Rebecer goal final do RViZ
        self.get_logger().info("New goal received! Planning...")
        self.goal_pose = msg.pose

        self.goal_position = [
            self.goal_pose.position.x,
            self.goal_pose.position.y,
        ]
        self.goal_orientation = self.get_yaw_from_quaternion(self.goal_pose.orientation)

    def distance_callback(self, msg: Float32MultiArray):
        self.get_logger().info("Received distances from lidar...")
        
        distances = list(msg.data)[:5]
        wind_rose = {k: d for d, k in zip(distances, ["NW", "NO", "NE", "ES", "WE"])}

        for missing_sector in ["SE", "SW", "SO"]:
            wind_rose[missing_sector] = 0.0

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
        self.robot_position = [t.transform.translation.x, t.transform.translation.y]
        self.robot_orientation = self.get_yaw_from_quaternion(t.transform.rotation)

        self.get_logger().info(f" Robot {self.robot_position} | {self.robot_orientation}...")

        if not None in [
            self.robot_position,
            self.robot_orientation,
            self.goal_position,
            self.goal_orientation,
        ]:
            
            # Get robot states in Benbouabdallah reference
            inputs = {
                "NW Distance": wind_rose["NW"],
                "NO Distance": wind_rose["NO"],
                "NE Distance": wind_rose["NE"],
                "ES Distance": wind_rose["ES"],
                "WE Distance": wind_rose["WE"],
            }
            results = fuzzy_nav.compute(inputs)
            D_star = results["D*"]
            alpha_star = results["Alpha*"]

            D, alpha = get_robot_goal_states(
                self.robot_position, 
                self.robot_orientation,
                self.goal_position,
            )

            Dd = 0.05
            v_max = 0.26
            omega_max = 1.82
            Kv, Kw = 0.3163, 1.6434

            # Control laws
            if D < wind_rose[alpha_to_wind_rose(alpha)]:
                v, omega = control_laws(D, Dd, alpha, Kv, Kw)

                self.create_marker(
                    xg = self.robot_position[0] + D * np.cos(self.robot_orientation - alpha),
                    yg = self.robot_position[1] + D * np.cos(self.robot_orientation - alpha),
                )

            else:
                v, omega = control_laws(D_star, Dd, alpha_star, Kv, Kw)

                self.create_marker(
                    xg = self.robot_position[0] + D * np.cos(self.robot_orientation - alpha_star),
                    yg = self.robot_position[1] + D * np.cos(self.robot_orientation - alpha_star),
                )

            v = np.clip(v, -v_max, v_max)
            omega = np.clip(omega, -omega_max, omega_max)

            msg = Twist()
            msg.linear.x = v
            msg.angular.z = omega

            self.twist_pub.publish(msg)

            self.get_logger().info(
                f"Linear Velocity: {v:.2f} | Angular Velocity: {omega:.2f}"
            )

            return 

        self.get_logger().info("Not enough info...")

    def create_marker(self, xg, yg):
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
        marker.color.r = 1.0
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
