import math
import argparse
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class SubgoalAnalyzer:
    def __init__(self, bag_path):
        self.bag_path = str(bag_path)
        self.REACHED_THRESHOLD = 0.5  # Success radius for a virtual subgoal
        self.ROBOT_RADIUS = 0.20       # Collision threshold
        
    def analyze(self):
        storage_options = StorageOptions(uri=self.bag_path, storage_id="sqlite3")
        converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        reader.set_filter(StorageFilter(topics=["/tf", "/lidar_sectors", "/fuzzy_subgoal"]))

        # State tracking
        current_subgoal = None
        subgoal_reached = False
        subgoal_had_collision = False
        
        # Aggregators
        total_subgoals_generated = 0
        successful_subgoals = 0
        robot_pose = None

        while reader.has_next():
            topic, data, _ = reader.read_next()

            # 1. Update Robot Pose
            if topic == "/tf":
                msg = deserialize_message(data, TFMessage)
                for t in msg.transforms:
                    if t.child_frame_id.lstrip("/") == "base_link":
                        robot_pose = (t.transform.translation.x, t.transform.translation.y)
                
                # Check if current subgoal is reached
                if current_subgoal and robot_pose and not subgoal_reached:
                    dist = math.sqrt((robot_pose[0] - current_subgoal[0])**2 + 
                                     (robot_pose[1] - current_subgoal[1])**2)
                    if dist < self.REACHED_THRESHOLD:
                        subgoal_reached = True

            # 2. Monitor Collisions during current subgoal travel
            if topic == "/lidar_sectors":
                msg = deserialize_message(data, Float32MultiArray)
                if min(msg.data) < self.ROBOT_RADIUS:
                    subgoal_had_collision = True

            # 3. Handle New Subgoal Generation
            if topic == "/fuzzy_subgoal":
                msg = deserialize_message(data, Marker)
                new_coords = (msg.pose.position.x, msg.pose.position.y)

                # If we had a previous subgoal, record its result before switching
                if current_subgoal is not None:
                    if subgoal_reached and not subgoal_had_collision:
                        successful_subgoals += 1
                
                # Reset for the new subgoal
                current_subgoal = new_coords
                total_subgoals_generated += 1
                subgoal_reached = False
                subgoal_had_collision = False

        # Calculate Rate
        success_rate = (successful_subgoals / total_subgoals_generated * 100) if total_subgoals_generated > 0 else 0
        
        return {
            "Total Subgoals Generated": total_subgoals_generated,
            "Successfully Reached": successful_subgoals,
            "Subgoal Success Rate (%)": round(success_rate, 2)
        }

class BagMetricsAggregator:
    def __init__(self, bag_path):
        self.bag_path = str(bag_path)
        
        # --- Threshold Constants ---
        self.ROBOT_RADIUS = 0.20        # From your physical setup
        self.NEAR_MISS_BUFFER = 0.15     # 15cm safety margin
        self.NEAR_COLLISION_VAL = self.ROBOT_RADIUS + self.NEAR_MISS_BUFFER # 0.35m
        self.SUCCESS_THRESHOLD = 0.25   # Distance to goal to stop timer

    def run_analysis(self):
        # ROS 2 Bag Reader Setup
        storage_options = StorageOptions(uri=self.bag_path, storage_id="sqlite3")
        converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        # Filtering for your specific topics
        topics = ["/tf", "/lidar_sectors", "/move_base_simple/goal"]
        reader.set_filter(StorageFilter(topics=topics))

        # Accumulators
        total_path_length = 0.0
        last_coords = None
        start_time = None
        end_time = None
        goal_coords = None
        
        near_collision_count = 0
        is_hazard_active = False

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            
            if start_time is None:
                start_time = timestamp

            # 1. Goal Extraction (Target Pose)
            if topic == "/move_base_simple/goal":
                msg = deserialize_message(data, PoseStamped)
                goal_coords = (msg.pose.position.x, msg.pose.position.y)

            # 2. Path Length & Time Tracking (from TF map -> base_link)
            if topic == "/tf":
                msg = deserialize_message(data, TFMessage)
                for t in msg.transforms:
                    if t.child_frame_id.lstrip("/") == "base_link":
                        curr_x = t.transform.translation.x
                        curr_y = t.transform.translation.y
                        
                        # Distance integration
                        if last_coords:
                            total_path_length += math.sqrt((curr_x - last_coords[0])**2 + 
                                                           (curr_y - last_coords[1])**2)
                        last_coords = (curr_x, curr_y)
                        
                        # Stop timer if goal is reached
                        if goal_coords:
                            dist_to_goal = math.sqrt((curr_x - goal_coords[0])**2 + 
                                                     (curr_y - goal_coords[1])**2)
                            if dist_to_goal < self.SUCCESS_THRESHOLD and end_time is None:
                                end_time = timestamp

            # 3. Near-Collision Situations (from Float32MultiArray)
            if topic == "/lidar_sectors":
                msg = deserialize_message(data, Float32MultiArray)
                # Your node order: NW, NO, NE, ES, WE
                min_dist = min(msg.data)
                
                # Logic: Enter hazard state -> increment; Exit hazard state -> reset
                if min_dist < self.NEAR_COLLISION_VAL:
                    if not is_hazard_active:
                        near_collision_count += 1
                        is_hazard_active = True
                else:
                    is_hazard_active = False

        # Final Calculations
        final_time = end_time if end_time else timestamp
        total_time_sec = (final_time - start_time) / 1e9

        return {
            "Total Path Length": f"{total_path_length:.2f} m",
            "Time to Goal": f"{total_time_sec:.2f} s",
            "Near-Collision Situations": near_collision_count
        }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Aggregate Navigation Metrics from Bag")
    parser.add_argument("bag", type=str, help="Path to ROS 2 bag")
    args = parser.parse_args()

    aggregator = BagMetricsAggregator(args.bag)
    results = SubgoalAnalyzer(args.bag).analyze()
    report = aggregator.run_analysis()

    print("\n" + "="*30)
    print("NAVIGATIONAL PERFORMANCE REPORT")
    print("="*30)
    for k, v in report.items():
        print(f"{k:25}: {v}")
    print("="*30)
    print("\n--- SUBGOAL SUCCESS REPORT ---")
    for k, v in results.items():
        print(f"{k}: {v}")