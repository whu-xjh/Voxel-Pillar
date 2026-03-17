#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Extract odometry position data from rosbag and save to txt file
Usage: python extract_odom_from_bag.py <bag_file> [output_file] [odom_topic]
"""

import rosbag
import sys
import argparse
from nav_msgs.msg import Odometry

def extract_odom_data(bag_file, output_file, odom_topic):
    """
    Extract odometry position data from rosbag

    Args:
        bag_file: Path to rosbag file
        output_file: Path to output txt file
        odom_topic: Odometry topic name
    """
    print(f"Processing bag file: {bag_file}")
    print(f"Searching for odom topic: {odom_topic}")

    bag = rosbag.Bag(bag_file)

    # Count messages
    total_msgs = bag.get_message_count(odom_topic)
    print(f"Found {total_msgs} odom messages")

    if total_msgs == 0:
        print(f"Error: Topic {odom_topic} not found")
        bag.close()
        return

    # Open output file
    with open(output_file, 'w') as f:
        # Write file header
        f.write("# timestamp x y z qx qy qz qw\n")

        count = 0
        for topic, msg, t in bag.read_messages(topics=[odom_topic]):
            # Extract position information
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z

            # Extract orientation quaternion
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            # Extract timestamp (seconds)
            timestamp = msg.header.stamp.to_sec()

            # Write to file
            f.write(f"{timestamp:.9f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

            count += 1
            if count % 1000 == 0:
                print(f"Processed: {count}/{total_msgs} messages")

    bag.close()
    print(f"\nDone! Extracted {count} odom data in total")
    print(f"Saved to: {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Extract odometry position data from rosbag')
    parser.add_argument('bag_file', help='Path to rosbag file')
    parser.add_argument('output_file', nargs='?', default='odom_data.txt',
                        help='Path to output txt file (default: odom_data.txt)')
    parser.add_argument('--topic', default='/novatel/oem7/odom',
                        help='Odometry topic name (default: /novatel/oem7/odom)')

    args = parser.parse_args()

    try:
        extract_odom_data(args.bag_file, args.output_file, args.topic)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
