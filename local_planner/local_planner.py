#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')
        self.get_logger().info("Hello from WOO")
        self.position = 10
        self.counter_ = 0.0
        self.vel_publisher = self.create_publisher(Float32, '/planning/desired_velocity', 10)  # Create a publisher for Float32 values, to desired velocity topic, Queue size of 10
        self.path_publisher = self.create_publisher(Path, '/planning/front_path/offset_path', 10)
        self.create_timer(1.0, self.publish_messages)  # Set a timer to run the timer_callback method every 5 seconds


    # def read_csv_column(self, input_file, column_name, lower_bound, upper_bound):
    #     with open(input_file, 'r') as csvfile:
    #         # Specify the delimiter as semicolon
    #         reader = csv.DictReader(csvfile, delimiter=';')
            
    #         for row in reader:
    #             value = float(row[column_name])

    #             if lower_bound <= value <= upper_bound:
    #                 print(value)

    def read_specific_columns(self, input_file):
        with open(input_file, 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=';')
            header = next(reader)  # Read the header row
            values = [(row[1], row[2], row[3]) for row in reader]
        return values[:10] #only return first 10 lines


    #define callback function to publish velocity from csv
    def publish_messages(self):
        vel_msg = Float32()
        vel_msg.data = float(self.counter_) # desired velocity placeholder
        path_msg = Path()

        for i in range(5):
            pose = PoseStamped()
            pose.pose.position.x = float(i)  # Set the x-coordinate of the position, needs to be float
            pose.pose.orientation.w = 1.0  # Set the w component of the quaternion (for orientation)
            # Append the pose to the list of poses in the path
            path_msg.poses.append(pose)
        
        self.vel_publisher.publish(vel_msg)
        self.path_publisher.publish(path_msg)

        path_info = self.format_path_message(path_msg)
        self.get_logger().info(f'Published Vel: {vel_msg.data} +\n Published Path: {path_info}')
        self.counter_ += 1.0 

    def format_path_message(self, path_msg):
        result = ""
        for i, pose_stamped in enumerate(path_msg.poses):
            result += f'Pose {i+1}:\n'
            result += f'Position (x, y, z): ({pose_stamped.pose.position.x}, {pose_stamped.pose.position.y}, {pose_stamped.pose.position.z})\n'
            result += f'Orientation (x, y, z, w): ({pose_stamped.pose.orientation.x}, {pose_stamped.pose.orientation.y}, {pose_stamped.pose.orientation.z}, {pose_stamped.pose.orientation.w})\n'
        return result

#for debugging
    def timer_callback(self):
        self.get_logger().info("Callback" + str(self.counter_))
        self.counter_ += 1

    def global_to_body_fixed(self, x, y, psi, x_init, y_init):
        # Define the 2D rotation matrix
        R = np.array([[np.cos(psi), -np.sin(psi), -1*x_init],
                    [np.sin(psi), np.cos(psi), -1*y_init],
                    [0, 0, 1]])
        
        # Create a column vector [x, y]
        p_global = np.array([[x], [y], [1]])
        
        # Apply the rotation
        p_body = np.dot(R, p_global)
        
        # Extract the resulting x_b and y_b
        x_b, y_b = p_body[0, 0], p_body[1, 0]
        
        return x_b, y_b

    def process_csv_data(self, data, x_init, y_init):
        for row in data:
            x = float(row[0])  # Assuming x_m is the first column
            y = float(row[1])  # Assuming y_m is the second column
            psi = float(row[2])  # Assuming psi_rad is the third column

            x_body, y_body = self.global_to_body_fixed(x, y, psi, x_init, y_init)
            print(f'Global Coordinates: x = {x}, y = {y}')
            print(f'Body-fixed Coordinates: x_b = {x_body}, y_b = {y_body}')

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    #test
 

    # Example usage
    input_file = os.path.expanduser('~/pair/src/local_planner/data/trajoutput.csv') #need to start from home dir and use os since nodes exist in diff environment
    column_name = 's_m'

    lower_bound = node.position
    upper_bound = lower_bound + 10

    #node.read_csv_column(input_file, column_name, lower_bound, upper_bound)
    data_x_y_psi = node.read_specific_columns(input_file)
    node.process_csv_data(data_x_y_psi)

    # # Test the transformation function
    # x_global = 3.0  # Example x-coordinate in global frame
    # y_global = 4.0  # Example y-coordinate in global frame
    # psi = np.pi/4  # Example yaw angle (45 degrees in radians)

    # x_body, y_body = node.global_to_body_fixed(x_global, y_global, psi)
    # print(f'Body-fixed coordinates: x_b = {x_body}, y_b = {y_body}')


    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
