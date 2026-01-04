#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
从rosbag中提取odom位置数据并保存到txt文件
用法: python extract_odom_from_bag.py <bag_file> [output_file] [odom_topic]
"""

import rosbag
import sys
import argparse
from nav_msgs.msg import Odometry

def extract_odom_data(bag_file, output_file, odom_topic):
    """
    从rosbag中提取odom位置数据
    
    Args:
        bag_file: rosbag文件路径
        output_file: 输出txt文件路径
        odom_topic: odom话题名称
    """
    print(f"正在处理bag文件: {bag_file}")
    print(f"查找odom话题: {odom_topic}")
    
    bag = rosbag.Bag(bag_file)
    
    # 统计消息数量
    total_msgs = bag.get_message_count(odom_topic)
    print(f"找到 {total_msgs} 条odom消息")
    
    if total_msgs == 0:
        print(f"错误: 未找到话题 {odom_topic}")
        bag.close()
        return
    
    # 打开输出文件
    with open(output_file, 'w') as f:
        # 写入文件头
        f.write("# timestamp x y z qx qy qz qw\n")
        
        count = 0
        for topic, msg, t in bag.read_messages(topics=[odom_topic]):
            # 提取位置信息
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            # 提取姿态四元数
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            # 提取时间戳（秒）
            timestamp = msg.header.stamp.to_sec()
            
            # 写入文件
            f.write(f"{timestamp:.9f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
            
            count += 1
            if count % 1000 == 0:
                print(f"已处理: {count}/{total_msgs} 条消息")
    
    bag.close()
    print(f"\n完成! 共提取 {count} 条odom数据")
    print(f"已保存到: {output_file}")

def main():
    parser = argparse.ArgumentParser(description='从rosbag中提取odom位置数据')
    parser.add_argument('bag_file', help='rosbag文件路径')
    parser.add_argument('output_file', nargs='?', default='odom_data.txt', 
                        help='输出txt文件路径 (默认: odom_data.txt)')
    parser.add_argument('--topic', default='/novatel/oem7/odom', 
                        help='odom话题名称 (默认: /novatel/oem7/odom)')
    
    args = parser.parse_args()
    
    try:
        extract_odom_data(args.bag_file, args.output_file, args.topic)
    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
