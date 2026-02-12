import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class GapDetectorNode(Node):
    def __init__(self):
        super().__init__('gap_detector_node')
        
        # Parameters based on TurtleBot3 (W_R = 287mm)
        self.robot_width = 0.287  
        self.ds = 1.5  # Safe distance threshold
        self.min_beam_n = 0 

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_gaps', 10)
        
        # NEW: Publish structured gap information for fuzzy planner
        self.gap_data_pub = self.create_publisher(Float32MultiArray, '/gap_data', 10)

    def calculate_n_min(self, angular_res):
        # Implementation of Eq. 1 from the article 
        term = (2 * (self.ds**2) - (self.robot_width**2)) / (2 * (self.ds**2))
        return int(np.ceil((1.0 / angular_res) * np.arccos(term)))

    def scan_callback(self, msg):
        if self.min_beam_n == 0:
            self.min_beam_n = self.calculate_n_min(msg.angle_increment)

        ranges = np.array(msg.ranges)
        # In Isaac Sim, distant obstacles may come as 'inf' or 'nan'
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0, neginf=0.0)

        gaps = []
        current_gap = []

        # Gap identification by continuity 
        for i, dist in enumerate(ranges):
            if dist > self.ds:
                current_gap.append(i)
            else:
                if len(current_gap) >= self.min_beam_n:
                    gaps.append(current_gap)
                current_gap = []
        
        if len(current_gap) >= self.min_beam_n:
            gaps.append(current_gap)

        # Publish markers for RViz
        self.publish_markers(gaps, msg, ranges)
        
        # NEW: Publish structured gap data for fuzzy planner
        self.publish_gap_data(gaps, msg, ranges)

    def publish_markers(self, gaps, msg, ranges):
        marker_array = MarkerArray()

        # Remove all previous gaps from RViz 
        clean_marker = Marker()
        clean_marker.header.frame_id = msg.header.frame_id
        clean_marker.action = Marker.DELETEALL
        marker_array.markers.append(clean_marker)

        for i, gap in enumerate(gaps):
            # Central angle of gap 
            avg_idx = int(np.mean(gap))
            angle = msg.angle_min + (avg_idx * msg.angle_increment)
            
            # Visualization distance based on actual reading
            actual_dist = ranges[avg_idx] if ranges[avg_idx] < self.ds else self.ds

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "gaps"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Virtual subgoal projected
            marker.points = [
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=actual_dist * np.cos(angle), y=actual_dist * np.sin(angle), z=0.0)
            ]
            
            marker.scale.x = 0.03  # Arrow body width
            marker.scale.y = 0.06  # Arrow tip width
            marker.scale.z = 0.06  # Arrow tip height
            marker.color.a = 0.8
            marker.color.r = 0.0
            marker.color.g = 1.0  # Green for available gap 
            marker.color.b = 0.2
            
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def publish_gap_data(self, gaps, msg, ranges):
        """
        Publishes gap data as Float32MultiArray with structure:
        [num_gaps, gap1_angle, gap1_depth, gap1_width, gap2_angle, gap2_depth, gap2_width, ...]
        
        - angle: central angle of the gap in radians (robot frame)
        - depth: average distance to obstacles in the gap
        - width: angular width of the gap in radians
        """
        gap_data = Float32MultiArray()
        
        if len(gaps) == 0:
            gap_data.data = [0.0]  # No gaps detected
        else:
            data_list = [float(len(gaps))]  # First element: number of gaps
            
            for gap in gaps:
                # Calculate gap properties
                start_idx = gap[0]
                end_idx = gap[-1]
                avg_idx = int(np.mean(gap))
                
                # Central angle (in robot frame)
                central_angle = msg.angle_min + (avg_idx * msg.angle_increment)
                
                # Gap depth: average distance within the gap
                gap_distances = ranges[gap]
                valid_distances = gap_distances[gap_distances > 0]
                if len(valid_distances) > 0:
                    gap_depth = np.mean(valid_distances)
                else:
                    gap_depth = self.ds
                
                # Gap width: angular span
                gap_width = (end_idx - start_idx + 1) * msg.angle_increment
                
                # Append to data list
                data_list.extend([central_angle, gap_depth, gap_width])
            
            gap_data.data = data_list
        
        self.gap_data_pub.publish(gap_data)

def main(args=None):
    rclpy.init(args=args)
    node = GapDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()