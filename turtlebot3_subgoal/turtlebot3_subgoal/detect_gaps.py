import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class GapDetectorNode(Node):
    def __init__(self):
        super().__init__('gap_detector_node')
        
        # Parâmetros baseados no TurtleBot3 (W_R = 287mm)
        self.robot_width = 0.287  
        self.ds = 1.5  # Aumentei o range para o Isaac Sim
        self.min_beam_n = 0 

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_gaps', 10)

    def calculate_n_min(self, angular_res):
        # Implementação da Eq. 1 do artigo 
        term = (2 * (self.ds**2) - (self.robot_width**2)) / (2 * (self.ds**2))
        return int(np.ceil((1.0 / angular_res) * np.arccos(term)))

    def scan_callback(self, msg):
        if self.min_beam_n == 0:
            self.min_beam_n = self.calculate_n_min(msg.angle_increment)

        ranges = np.array(msg.ranges)
        # No Isaac Sim, obstáculos distantes podem vir como 'inf' ou 'nan'
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0, neginf=0.0)

        gaps = []
        current_gap = []

        # Identificação de gaps por continuidade 
        for i, dist in enumerate(ranges):
            if dist > self.ds:
                current_gap.append(i)
            else:
                if len(current_gap) >= self.min_beam_n:
                    gaps.append(current_gap)
                current_gap = []
        
        if len(current_gap) >= self.min_beam_n:
            gaps.append(current_gap)

        self.publish_markers(gaps, msg, ranges)

    def publish_markers(self, gaps, msg, ranges):
        marker_array = MarkerArray()

        # Remove todos os gaps anteriores do RViz 
        clean_marker = Marker()
        clean_marker.header.frame_id = msg.header.frame_id
        clean_marker.action = Marker.DELETEALL
        marker_array.markers.append(clean_marker)

        for i, gap in enumerate(gaps):
            # Ângulo central do gap 
            avg_idx = int(np.mean(gap))
            angle = msg.angle_min + (avg_idx * msg.angle_increment)
            
            # Distância de visualização baseada na leitura real [cite: 379]
            actual_dist = ranges[avg_idx] if ranges[avg_idx] < self.ds else self.ds

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "gaps"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Subgoal virtual projetado
            marker.points = [
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=actual_dist * np.cos(angle), y=actual_dist * np.sin(angle), z=0.0)
            ]
            
            marker.scale.x = 0.03 # Largura do corpo da seta
            marker.scale.y = 0.06 # Largura da ponta
            marker.scale.z = 0.06 # Altura da ponta
            marker.color.a = 0.8
            marker.color.r = 0.0
            marker.color.g = 1.0 # Verde para gap disponível 
            marker.color.b = 0.2
            
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GapDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()