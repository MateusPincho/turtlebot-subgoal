import os
import yaml
import math
import argparse
import numpy as np
from PIL import Image
from pathlib import Path
import matplotlib.pyplot as plt


from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rclpy
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid


def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def draw_robot(x, y, yaw, size, ax, color="red"):
    height = size * 1.25
    width = size * 1.0

    # Points in local frame (tip, rear-left, rear-right)
    pts = np.array([[height, 0], [-height / 2, width / 2], [-height / 2, -width / 2]])

    # Rotation matrix
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])

    # Rotate and translate points
    rotated_pts = (R @ pts.T).T + np.array([x, y])

    # Create polygon
    polygon = plt.Polygon(rotated_pts, fc=color, ec=color, alpha=1.0, zorder=5)
    ax.add_patch(polygon)


def plot_ros2_occupancy_grid(yaml_path):
    with open(yaml_path, "r") as f:
        map_metadata = yaml.safe_load(f)

    image_file = map_metadata["image"]
    resolution = map_metadata["resolution"]
    origin = map_metadata["origin"]

    yaml_dir = os.path.dirname(yaml_path)
    pgm_path = os.path.join(yaml_dir, image_file)

    pgm_img = Image.open(pgm_path)
    occupancy_grid = np.array(pgm_img)

    height, width = occupancy_grid.shape
    left = origin[0]
    right = origin[0] + (width * resolution)
    bottom = origin[1]
    top = origin[1] + (height * resolution)
    extent = [left, right, bottom, top]

    # We use 'gray' colormap; ROS usually sets 255 (white) as free, 0 (black) as occupied
    # origin='lower' is critical because ROS coordinates start at bottom-left
    plt.imshow(occupancy_grid, cmap="gray", extent=extent, origin="lower")


def get_rosbag_options(path, serialization_format="cdr"):
    storage_options = StorageOptions(uri=path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )
    return storage_options, converter_options


def extract_and_plot_map(storage_options, converter_options, map_topic, ax):
    """
    Finds the OccupancyGrid in the bag and plots it by creating its own reader instance.
    """
    print(f"[INFO] Searching for map on topic: {map_topic}")

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Apply filter specifically for the map topic
    # We try both the provided name and a version without the leading slash if necessary
    alt_map_topic = map_topic[1:] if map_topic.startswith("/") else "/" + map_topic
    reader.set_filter(StorageFilter(topics=[map_topic, alt_map_topic]))

    latest_map_msg = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == map_topic or topic == alt_map_topic:
            latest_map_msg = deserialize_message(data, OccupancyGrid)

    if latest_map_msg is None:
        print(
            f"[WARNING] No map found on topic {map_topic}. Plotting without background."
        )
        return False

    # Extract metadata
    res = latest_map_msg.info.resolution
    width = latest_map_msg.info.width
    height = latest_map_msg.info.height
    origin_x = latest_map_msg.info.origin.position.x
    origin_y = latest_map_msg.info.origin.position.y

    # Convert data (-1 to 100) to numpy array
    map_data = np.array(latest_map_msg.data).reshape((height, width))

    # Create display map
    # -1: Unknown -> mapped to a distinct gray (200)
    # 0 to 100: Probability -> mapped to grayscale 255 (white) to 0 (black)
    display_map = np.zeros((height, width), dtype=np.uint8)

    # Handle the probability gradient (0-100)
    # Formula: 255 - (probability * 2.55)
    # This ensures 0% = 255 (white) and 100% = 0 (black)
    prob_mask = map_data >= 0
    display_map[prob_mask] = (255 - (map_data[prob_mask] * 2.55)).astype(np.uint8)

    # Explicitly set Unknown (-1) to a mid-gray value
    display_map[map_data == -1] = 200

    extent = [origin_x, origin_x + (width * res), origin_y, origin_y + (height * res)]

    ax.imshow(
        display_map, cmap="gray", extent=extent, origin="lower", alpha=1.0, zorder=1
    )
    print(f"[INFO] Map rendered: {width}x{height} at {res}m/px")
    return extent


def plot_pose_trajectory(
    bag_path,
    robot_pose_tf_topic="/tf",
    target_topic="/move_base_simple/goal",
    map_topic="/map",
):
    print(f"[INFO] Reading ROS 2 bag from: {bag_path}")
    print(f"[INFO] Robot pose (TF) topic: {robot_pose_tf_topic}")
    print(f"[INFO] Target topic: {target_topic}")

    if not rclpy.ok():
        rclpy.init()

    try:
        storage_options, converter_options = get_rosbag_options(bag_path)
        reader = SequentialReader()

        # FIX: Some versions of rosbag2_py do not accept the filter in the open() call.
        # We open the bag first, then set the filter separately.
        reader.open(storage_options, converter_options)

        storage_filter = StorageFilter(topics=[target_topic, robot_pose_tf_topic])
        reader.set_filter(storage_filter)

    except Exception as e:
        print(f"[ERROR] Could not open rosbag: {e}")
        return

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 10))

    map_extent = extract_and_plot_map(storage_options, converter_options, map_topic, ax)

    # Data for Target (PoseStamped)
    target_x, target_y = [], []

    # Data for Robot (from TF map -> base_link)
    robot_x, robot_y, robot_yaw = [], [], []

    # Get message types
    # Target is usually PoseStamped
    try:
        target_msg_type = get_message("geometry_msgs/msg/PoseStamped")
    except:
        print("[ERROR] Could not load PoseStamped message type.")
        return

    print("Extracting trajectory data...")
    while reader.has_next():
        topic, data, _ = reader.read_next()

        # Extract Target PoseStamped
        if topic in [
            target_topic,
            target_topic.lstrip("/"),
            "/" + target_topic.lstrip("/"),
        ]:
            try:
                msg = deserialize_message(data, target_msg_type)
                target_x.append(msg.pose.position.x)
                target_y.append(msg.pose.position.y)
            except:
                continue

        # Extract Robot Pose from TF
        elif topic in ["/tf", "/tf_static"]:
            try:
                tf_msg = deserialize_message(data, TFMessage)
                for t in tf_msg.transforms:
                    # Check for common frame names
                    parent = t.header.frame_id.lstrip("/")
                    child = t.child_frame_id.lstrip("/")

                    if parent in ["map", "odom"] and child in [
                        "base_link",
                        "base_footprint",
                    ]:
                        robot_x.append(t.transform.translation.x)
                        robot_y.append(t.transform.translation.y)
                        robot_yaw.append(get_yaw_from_quaternion(t.transform.rotation))
            except:
                continue

    if not target_x and not robot_x:
        print("[WARNING] No valid messages were extracted.")
        return

    print(
        f"[INFO] Extracted {len(target_x)} target points and {len(robot_x)} robot TF points."
    )

    if robot_x:
        ax.plot(
            robot_x,
            robot_y,
            color="blue",
            alpha=1.0,
            linestyle="solid",
            linewidth=2,
            zorder=2,
            label="Path",
        )

        # Draw robot arrows at regular intervals
        interval = max(1, len(robot_x) // 20)
        for i in range(int(len(robot_x) * 0.3), len(robot_x), interval):
            draw_robot(robot_x[i], robot_y[i], robot_yaw[i], 0.075, ax)

    if robot_x:
        ax.scatter(
            robot_x[0],
            robot_y[0],
            color="green",
            s=75,
            marker="o",
            zorder=3,
            label="Start",
        )

    if target_x:
        ax.scatter(
            target_x[-1],
            target_y[-1],
            color="red",
            s=75,
            marker="X",
            zorder=3,
            label="Goal",
        )

    # Styling
    ax.set_title(f"Robot Path", fontsize=16, fontweight="bold")
    ax.set_xlabel("X [m]", fontsize=12)
    ax.set_ylabel("Y [m]", fontsize=12)

    # Tight fitting: Use the map extent if available, otherwise default to data bounds
    if map_extent:
        ax.set_xlim([map_extent[0], map_extent[1]])
        ax.set_ylim([map_extent[2], map_extent[3]])
    elif robot_x or target_x:
        # Fallback to auto-scaling if no map but data exists
        ax.autoscale(enable=True, axis="both", tight=True)
    else:
        # Total fallback
        ax.set_xlim([-5.0, 5.0])
        ax.set_ylim([-5.0, 5.0])

    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.set_aspect("equal", adjustable="box")

    plt.tight_layout()

    output_filename = f"{Path(bag_path).stem}.png"
    plt.savefig(output_filename)
    print(f"\nPlot saved successfully to {output_filename}")

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot the X-Y trajectory from a specific topic in a ROS 2 rosbag."
    )
    parser.add_argument(
        "bag_path",
        type=str,
        help="The file path to the ROS 2 bag directory (for v3 bags) or the .db3 file (for v2 bags).",
    )

    args = parser.parse_args()

    plot_pose_trajectory(args.bag_path)
