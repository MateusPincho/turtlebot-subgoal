import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float32MultiArray

import math

class LidarSectorNode(Node):
    def __init__(self):
        super().__init__('lidar_sector_node')
        
        # Subscriber (Sensors - Best Effort)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile_sensor_data)

        # Publish Markers for visualization in RVIZ
        self.marker_pub = self.create_publisher(MarkerArray, 'sector_markers', 10)
        
        # Publish the Lidar Sectors distance
        self.sector_pub = self.create_publisher(Float32MultiArray, '/lidar_sectors', 10)

        self.get_logger().info('Lidar Sector Node with Visualization started.')

        # Define center angles for visualization (in radians)
        self.sector_angles = {
            "NO": 0.0,             # North (Front)
            "NW": math.pi / 4,     # North-West (+45 deg)
            "WE": math.pi / 2,     # West (+90 deg)
            "SW": 3 * math.pi / 4, # South-West (+135 deg)
            "SO": math.pi,         # South (180 deg)
            "SE": -3 * math.pi / 4,# South-East (-135 deg)
            "ES": -math.pi / 2,    # East (-90 deg)
            "NE": -math.pi / 4     # North-East (-45 deg)
        }

    def listener_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        def angle_to_index(angle_rad):
            # logic: (angle - min) / inc
            return int((angle_rad - angle_min) / angle_increment)

        sector_step = math.pi / 4.0  # 45 degrees
        half_sector = sector_step / 2.0 

        # --- SECTOR CALCULATION ---

        sector_data = {}

        # NO (North) 
        idx_no_start_1 = angle_to_index(2 * math.pi - half_sector)
        idx_no_end_1   = len(ranges)
        idx_no_start_2 = 0
        idx_no_end_2   = angle_to_index(half_sector)
        sector_data["NO"] = ranges[idx_no_start_1:idx_no_end_1] + ranges[idx_no_start_2:idx_no_end_2]

        # Others Directions
        sectors_order = ["NW", "WE", "SW", "SO", "SE", "ES", "NE"]
        current_angle = half_sector
        for sector_name in sectors_order:
            start_idx = angle_to_index(current_angle)
            end_idx = angle_to_index(current_angle + sector_step)
            start_idx = max(0, min(start_idx, len(ranges)))
            end_idx = max(0, min(end_idx, len(ranges)))
            sector_data[sector_name] = ranges[start_idx:end_idx]
            current_angle += sector_step

        # --- DATA PUBLICATION ---
        
        # Helper to find min distance for data logic
        # If clear, we return a large value (10.0m) so fuzzy logic sees it as "Far"
        def get_min_dist_val(data_list):
            valid = [x for x in data_list if not math.isnan(x) and not math.isinf(x) and x > msg.range_min]
            return min(valid) if valid else 10.0 

        sector_msg = Float32MultiArray()
        # Order: NW, NO, NE, ES, WE
        sector_msg.data = [
            get_min_dist_val(sector_data["NW"]),
            get_min_dist_val(sector_data["NO"]),
            get_min_dist_val(sector_data["NE"]),
            get_min_dist_val(sector_data["ES"]),
            get_min_dist_val(sector_data["WE"])   
        ]
        self.sector_pub.publish(sector_msg)

        # --- VISUALIZATION GENERATION ---
        marker_array = MarkerArray()
        
        # Helper to find min distance for visual logic
        # If clear, we default to 2.0m just so the arrow is visible but not huge
        def get_min_dist(data_list):
            valid = [x for x in data_list if not math.isnan(x) and not math.isinf(x) and x > msg.range_min]
            return min(valid) if valid else 3.0 # Default length if clear (3m)

        marker_id = 0
        
        for name, data in sector_data.items():
            dist = get_min_dist(data)
            
            # Is it an obstacle or clear space?
            is_obstacle = dist < 3.0 # Threshold for coloring
            
            # Calculate endpoint for the line
            angle = self.sector_angles[name]
            end_point = Point()
            end_point.x = dist * math.cos(angle)
            end_point.y = dist * math.sin(angle)
            end_point.z = 0.0

            # 1. Create LINE Marker (The Ray)
            line_marker = Marker()
            line_marker.header = msg.header # Use same frame/time as scan
            line_marker.ns = "sector_rays"
            line_marker.id = marker_id
            marker_id += 1
            line_marker.type = Marker.ARROW
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05 # Shaft diameter
            line_marker.scale.y = 0.1  # Head diameter
            line_marker.scale.z = 0.1  # Head length
            
            # Color: Red if obstacle, Green if clear
            line_marker.color.a = 1.0
            if is_obstacle:
                line_marker.color.r = 1.0; line_marker.color.g = 0.0; line_marker.color.b = 0.0
            else:
                line_marker.color.r = 0.0; line_marker.color.g = 1.0; line_marker.color.b = 0.0

            # Points: Start (0,0) -> End
            p_start = Point(); p_start.x = 0.0; p_start.y = 0.0; p_start.z = 0.0
            line_marker.points.append(p_start)
            line_marker.points.append(end_point)
            
            marker_array.markers.append(line_marker)

            # 2. Create TEXT Marker (The Label "NO", "NW", etc)
            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = "sector_names"
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = end_point.x * 1.1 # Slightly past the arrow
            text_marker.pose.position.y = end_point.y * 1.1
            text_marker.pose.position.z = 0.2
            text_marker.scale.z = 0.2 # Text height
            text_marker.color.a = 1.0; text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0
            text_marker.text = f"{name}\n{dist:.2f}m"
            
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarSectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()