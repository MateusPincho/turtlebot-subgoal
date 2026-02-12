import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import math
import numpy as np


class FuzzySystem:
    def __init__(self):
        self.variables = {}
        self.rules = []

    def add_variable(self, name, range_min, range_max, resolution=1000):
        self.variables[name] = {
            "range": np.linspace(range_min, range_max, resolution),
            "terms": {},
        }

    def add_term(self, var_name, term_name, mf_type, params, saturate=None):
        universe = self.variables[var_name]["range"]

        if mf_type == "Triangular":
            a, b, c = params
            mf = np.maximum(
                0,
                np.minimum(
                    (universe - a) / (b - a + 1e-9), (c - universe) / (c - b + 1e-9)
                ),
            )
            peak_start, peak_end = b, b

        elif mf_type == "Trapezoidal":
            a, b, c, d = params
            term1 = (universe - a) / (b - a + 1e-9)
            term2 = (d - universe) / (d - c + 1e-9)
            mf = np.maximum(0, np.minimum(np.minimum(term1, 1), term2))
            peak_start, peak_end = b, c

        elif mf_type == "Gaussian":
            mean, sigma = params
            mf = np.exp(-0.5 * ((universe - mean) / (sigma + 1e-9)) ** 2)
            peak_start, peak_end = mean, mean

        else:
            raise ValueError(
                "Unsupported MF type. Use 'Triangular', 'Trapezoidal', or 'Gaussian'."
            )

        if saturate == "Left":
            mf = np.where(universe <= peak_start, np.maximum(mf, 1.0), mf)
        elif saturate == "Right":
            mf = np.where(universe >= peak_end, np.maximum(mf, 1.0), mf)

        self.variables[var_name]["terms"][term_name] = mf

    def _get_membership_value(self, var_name, term_name, value):
        universe = self.variables[var_name]["range"]
        mf = self.variables[var_name]["terms"][term_name]
        return np.interp(value, universe, mf)

    def add_rule(self, antecedents, consequents, operator="and"):
        if isinstance(consequents, tuple):
            consequents = [consequents]
        self.rules.append(
            {"if": antecedents, "then": consequents, "op": operator.lower()}
        )

    def compute(self, inputs):
        aggregated_outputs = {}

        for rule in self.rules:
            degrees = [
                self._get_membership_value(var, term, inputs.get(var, 0))
                for var, term in rule["if"]
            ]
            activation = min(degrees) if rule["op"] == "and" else max(degrees)

            for out_var, out_term in rule["then"]:
                out_mf = self.variables[out_var]["terms"][out_term]
                clipped_mf = np.minimum(activation, out_mf)

                if out_var not in aggregated_outputs:
                    aggregated_outputs[out_var] = np.zeros_like(out_mf)

                aggregated_outputs[out_var] = np.maximum(
                    aggregated_outputs[out_var], clipped_mf
                )

        results = {}
        for var_name, final_mf in aggregated_outputs.items():
            universe = self.variables[var_name]["range"]
            sum_mu = np.sum(final_mf)
            results[var_name] = (
                np.sum(final_mf * universe) / sum_mu
                if sum_mu > 0
                else np.mean(universe)
            )
        return results


# ============================================================================
# GAP-BASED FUZZY NAVIGATION SYSTEM
# ============================================================================

fuzzy_gap_eval = FuzzySystem()

# INPUT 1: Goal Proximity (Delta Theta) - How aligned the gap is with the goal
fuzzy_gap_eval.add_variable("Goal Proximity", 0.0, np.pi)
fuzzy_gap_eval.add_term("Goal Proximity", "Small", "Trapezoidal", [0.0, 0.0, 0.3, 1.6], saturate="Left")
fuzzy_gap_eval.add_term("Goal Proximity", "Medium", "Triangular", [0.4, 0.9, 1.4])
fuzzy_gap_eval.add_term("Goal Proximity", "Large", "Trapezoidal", [1.2, 1.8, np.pi, np.pi], saturate="Right")

# INPUT 2: Gap Amplitude (W) - Angular width of the gap
fuzzy_gap_eval.add_variable("Gap Amplitude", 0.0, np.pi)
fuzzy_gap_eval.add_term("Gap Amplitude", "Narrow", "Trapezoidal", [0.0, 0.0, 0.3, 0.6], saturate="Left")
fuzzy_gap_eval.add_term("Gap Amplitude", "Wide", "Trapezoidal", [0.4, 0.8, np.pi, np.pi], saturate="Right")

# INPUT 3: Gap Depth (d_gap) - Free space ahead in the gap
fuzzy_gap_eval.add_variable("Gap Depth", 0.0, 10.0)
fuzzy_gap_eval.add_term("Gap Depth", "Short", "Trapezoidal", [0.0, 0.0, 1.0, 2.0], saturate="Left")
fuzzy_gap_eval.add_term("Gap Depth", "Long", "Trapezoidal", [1.5, 3.0, 10.0, 10.0], saturate="Right")

# OUTPUT: Fitness (Score) - Desirability of the gap
fuzzy_gap_eval.add_variable("Fitness", 0.0, 1.0)
fuzzy_gap_eval.add_term("Fitness", "Very Low", "Trapezoidal", [0.0, 0.0, 0.1, 0.25], saturate="Left")
fuzzy_gap_eval.add_term("Fitness", "Low", "Triangular", [0.15, 0.3, 0.45])
fuzzy_gap_eval.add_term("Fitness", "Medium", "Triangular", [0.35, 0.5, 0.65])
fuzzy_gap_eval.add_term("Fitness", "High", "Triangular", [0.55, 0.7, 0.85])
fuzzy_gap_eval.add_term("Fitness", "Very High", "Trapezoidal", [0.75, 0.9, 1.0, 1.0], saturate="Right")


# FUZZY RULES
fuzzy_gap_eval.add_rule(
    [("Goal Proximity", "Small"), ("Gap Depth", "Long")],
    ("Fitness", "Very High"), operator="and"
)

fuzzy_gap_eval.add_rule(
    [("Goal Proximity", "Small"), ("Gap Depth", "Short")],
    ("Fitness", "High"), operator="and"
)

fuzzy_gap_eval.add_rule(
    [("Goal Proximity", "Medium"), ("Gap Depth", "Long"), ("Gap Amplitude", "Wide")],
    ("Fitness", "High"), operator="and"
)

fuzzy_gap_eval.add_rule(
    [("Goal Proximity", "Medium"), ("Gap Depth", "Long"), ("Gap Amplitude", "Narrow")],
    ("Fitness", "Medium"), operator="and"
)

fuzzy_gap_eval.add_rule(
    [("Goal Proximity", "Medium"), ("Gap Depth", "Short")],
    ("Fitness", "Low"), operator="and"
)

fuzzy_gap_eval.add_rule(
    [("Goal Proximity", "Large"), ("Gap Depth", "Long"), ("Gap Amplitude", "Wide")],
    ("Fitness", "Medium"), operator="and"
)

fuzzy_gap_eval.add_rule(
    [("Goal Proximity", "Large"), ("Gap Depth", "Long"), ("Gap Amplitude", "Narrow")],
    ("Fitness", "Low"), operator="and"
)

fuzzy_gap_eval.add_rule(
    [("Goal Proximity", "Large"), ("Gap Depth", "Short")],
    ("Fitness", "Very Low"), operator="and"
)

fuzzy_gap_eval.add_rule(
    [("Goal Proximity", "Small"), ("Gap Amplitude", "Wide")],
    ("Fitness", "High"), operator="and"
)

fuzzy_gap_eval.add_rule(
    [("Goal Proximity", "Large"), ("Gap Amplitude", "Narrow")],
    ("Fitness", "Very Low"), operator="and"
)


def get_robot_goal_states(robot_position, robot_orientation, goal_position):
    """Calculate distance and angle to goal."""
    dx = goal_position[0] - robot_position[0]
    dy = goal_position[1] - robot_position[1]
    D = np.sqrt(dx**2 + dy**2)
    theta_goal = np.arctan2(dy, dx)
    alpha = wrap_to_pi(theta_goal - robot_orientation)
    return D, alpha

def control_laws(D, Dd, alpha, Kv, Kw):
    ed = D - Dd
    v = Kv * ed * np.cos(alpha)
    omega = -Kw * alpha - v * np.sin(alpha) / D

    return v, omega

def wrap_to_pi(angle):
    """Normalize angle to [-π, π]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class FuzzyGapPlanner(Node):
    def __init__(self):
        super().__init__("fuzzy_gap_planner_node")

        # Subscribers
        self.rviz_goal_sub = self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self.goal_callback, 10
        )
        self.gap_data_sub = self.create_subscription(
            Float32MultiArray, "/gap_data", self.gap_callback, 10
        )

        # Publishers
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(Marker, "/fuzzy_subgoal", 10)

        # TF for localization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State variables
        self.robot_position = None
        self.robot_orientation = None
        self.goal_position = None
        self.gap_data = None

        # Control parameters
        self.d_lookahead = 1.0
        self.goal_tolerance = 0.15
        
        # DEBUG MODE - set to False to reduce logging
        self.debug = True
        
        self.get_logger().info("Fuzzy Gap Planner initialized. Waiting for goal...")

    def goal_callback(self, msg: PoseStamped):
        """Receive final goal from RViz."""
        self.get_logger().info("New goal received!")
        self.goal_position = [msg.pose.position.x, msg.pose.position.y]

    def gap_callback(self, msg: Float32MultiArray):
        """Receive gap data from gap detector."""
        self.gap_data = list(msg.data)
        
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF not ready: {e}")
            return

        self.robot_position = [t.transform.translation.x, t.transform.translation.y]
        self.robot_orientation = self.get_yaw_from_quaternion(t.transform.rotation)

        if self.goal_position is not None and self.gap_data is not None:
            self.navigate()

    def navigate(self):
        """Main navigation logic using fuzzy gap evaluation."""
        # Check if goal is reached
        D_goal, alpha_goal = get_robot_goal_states(
            self.robot_position, self.robot_orientation, self.goal_position
        )
        
        if self.debug:
            self.get_logger().info("=" * 80)
            self.get_logger().info(f"Distance to goal: {D_goal:.2f}m, Angle: {np.degrees(alpha_goal):.1f}°")
        
        if D_goal < self.goal_tolerance:
            self.get_logger().info("Goal reached! Stopping.")
            self.publish_velocity(0.0, 0.0)
            return

        # Parse gap data
        num_gaps = int(self.gap_data[0])
        
        if num_gaps == 0:
            self.get_logger().warn("No gaps detected! Emergency stop.")
            self.publish_velocity(0.0, 0.0)
            return

        # CRITICAL FIX: When no obstacles, go directly to goal
        # Check if there's a clear path to goal
        if num_gaps == 1:
            # Only one gap - check if it's aligned with goal
            gap_angle = self.gap_data[1]
            gap_depth = self.gap_data[2]
            
            # If the single gap is roughly aligned with goal direction, use goal directly
            if abs(wrap_to_pi(gap_angle - alpha_goal)) < 0.5 and gap_depth > D_goal:
                if self.debug:
                    self.get_logger().info("Clear path to goal detected! Using direct navigation.")
                
                # Use goal directly
                alpha = alpha_goal
                e = min(D_goal, self.d_lookahead)
                
                # Apply control law
                Kv, Kw = 0.4, 2.0
                Dd = 0.05
                v, omega = control_laws(e, Dd, alpha, Kv, Kw)

                v_max = 0.26
                omega_max = 1.82
                v = np.clip(v, -v_max, v_max)
                omega = np.clip(omega, -omega_max, omega_max)

                self.publish_velocity(v, omega)

                # Publish goal marker
                goal_x = self.goal_position[0]
                goal_y = self.goal_position[1]
                self.publish_marker(goal_x, goal_y, 1.0)

                if self.debug:
                    self.get_logger().info(f"Direct navigation: v={v:.2f}, ω={omega:.2f}")
                return

        # Evaluate each gap using fuzzy logic
        best_gap = None
        best_fitness = -1.0

        if self.debug:
            self.get_logger().info(f"\nEvaluating {num_gaps} gaps:")

        for i in range(num_gaps):
            idx = 1 + i * 3
            gap_angle = self.gap_data[idx]
            gap_depth = self.gap_data[idx + 1]
            gap_width = self.gap_data[idx + 2]

            # CRITICAL: Calculate goal proximity correctly
            # gap_angle is in robot frame, alpha_goal is also in robot frame
            # So the difference directly gives us the proximity
            delta_theta = abs(wrap_to_pi(gap_angle - alpha_goal))

            # Fuzzy inputs
            inputs = {
                "Goal Proximity": delta_theta,
                "Gap Amplitude": gap_width,
                "Gap Depth": gap_depth,
            }

            results = fuzzy_gap_eval.compute(inputs)
            fitness = results["Fitness"]

            if self.debug:
                self.get_logger().info(
                    f"  Gap {i}: angle={np.degrees(gap_angle):+.1f}° (vs goal {np.degrees(alpha_goal):+.1f}°), "
                    f"Δθ={np.degrees(delta_theta):.1f}°, depth={gap_depth:.2f}m, "
                    f"width={np.degrees(gap_width):.1f}°, fitness={fitness:.3f}"
                )

            if fitness > best_fitness:
                best_fitness = fitness
                best_gap = {
                    "angle": gap_angle,
                    "depth": gap_depth,
                    "width": gap_width,
                    "fitness": fitness,
                }

        # Generate subgoal from best gap
        if best_gap is not None:
            if self.debug:
                self.get_logger().info(f"\nSelected gap: fitness={best_fitness:.3f}")

            # CRITICAL: Use gap angle directly as it's in robot frame
            alpha = wrap_to_pi(best_gap["angle"])
            e = min(best_gap["depth"], self.d_lookahead)

            # Apply control law
            Kv, Kw = 0.4, 2.0
            Dd = 0.05
            v, omega = control_laws(e, Dd, alpha, Kv, Kw)

            v_max = 0.26
            omega_max = 1.82
            v = np.clip(v, -v_max, v_max)
            omega = np.clip(omega, -omega_max, omega_max)

            self.publish_velocity(v, omega)

            # Publish subgoal marker
            subgoal_x = self.robot_position[0] + e * np.cos(self.robot_orientation + alpha)
            subgoal_y = self.robot_position[1] + e * np.sin(self.robot_orientation + alpha)
            self.publish_marker(subgoal_x, subgoal_y, best_fitness)

            if self.debug:
                self.get_logger().info(f"v={v:.2f}, ω={omega:.2f}, α={np.degrees(alpha):.1f}°, e={e:.2f}m")



    def publish_velocity(self, v, omega):
        """Publish velocity command."""
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self.twist_pub.publish(msg)

    def publish_marker(self, x, y, fitness):
        """Publish subgoal marker in RViz."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0
        if fitness > 0.7:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
        elif fitness > 0.4:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0

        self.marker_pub.publish(marker)

    @staticmethod
    def get_yaw_from_quaternion(q):
        """Extract yaw angle from quaternion."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyGapPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()