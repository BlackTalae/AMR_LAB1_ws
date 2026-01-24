import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import sys
import argparse
from sensor_msgs.msg import JointState, Imu, LaserScan

def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options

def main():
    parser = argparse.ArgumentParser(description='Read ROS 2 bag data')
    parser.add_argument('--bag', required=True, help='Path to the bag directory')
    args = parser.parse_args()

    bag_path = args.bag
    
    storage_options, converter_options = get_rosbag_options(bag_path)
    reader = rosbag2_py.SequentialReader()
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        return

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    print("Reading bag...")
    
    # Counter for topics
    counts = {}

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        if topic not in counts:
            counts[topic] = 0
            # Print first message of each topic as a sample
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            print(f"\n[First Message] Topic: {topic}, Type: {type_map[topic]}")
            print(msg)

        counts[topic] += 1

    print("\nSummary:")
    for topic, count in counts.items():
        print(f"Topic: {topic}, Count: {count}")

if __name__ == '__main__':
    main()
