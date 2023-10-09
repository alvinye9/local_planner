#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point
import tf2_ros
import tf2_py
from tf2_geometry_msgs import do_transform_point
import numpy as np
from scipy.spatial import cKDTree
import csv
import os


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_global = self.create_publisher(Path, '/global_path', 10)
        self.publisher_local = self.create_publisher(Path, '/local_path', 10)

        self.global_path = Path()
        self.global_path.header.frame_id = 'map'

        # Positions will be used to build KDTree
        self.positions = []

        
        
        input_file = os.path.expanduser('~/pair/src/local_planner/data/trajoutput.csv') #need to start from home dir and use os since nodes exist in diff environment
        data_x_y_psi = self.read_specific_columns(input_file)
        self.assign_data_to_path(data_x_y_psi)
        #self.straight_line()

        self.publisher_global.publish(self.global_path) 
    
        # KDTree allows for quick lookup of points
        self.kdtree = cKDTree(self.positions)

        # Setup storage for TF buffer and define listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform = None

        timer_period = 0.02  # seconds
        # Timer to call our publish function
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def straight_line(self):
        for i in range(1000):
            x = i * 0.1
            y = 0.0
            pose = PoseStamped()
            pose.header.frame_id = 'map' #tells us which frame this path is in
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.global_path.poses.append(pose)
            self.positions.append([x, y])
    
    def assign_data_to_path(self, data):
        #print("assigning csv data to path msg")
        #print(data)

        #WORK ON ADDING ORIENTATION?
        for x, y, psi in data:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.global_path.poses.append(pose)
            self.positions.append([x, y])


    def read_specific_columns(self, input_file):
        with open(input_file, 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=';')
            header = next(reader)  # Read the header row
            values = [(float(row[1]), float(row[2]), float(row[3])) for row in reader]
            #print(values)
        return values #return all the values within these columns
    

    def timer_callback(self):
        # Try to get transform vehicle->map, return if fails
        try:
            self.transform = self.tf_buffer.lookup_transform("map", "vehicle", rclpy.time.Time())
        except:
            return

        orig = PointStamped()
        orig.point.x, orig.point.y = 0.0, 0.0

        # Transforming (0, 0) in car frame to global frame gives global car coordinates
        # there has to be a better way to do this
        car_loc = do_transform_point(orig, self.transform)
        x, y = car_loc.point.x, car_loc.point.y

        # Find closest point on path to global point
        _, idx = self.kdtree.query([x, y])

        local_path = Path()
        local_path.header.frame_id = 'vehicle'
        local_path.header.stamp = self.get_clock().now().to_msg()

        # Transform poses in global path ahead of vehicle into vehicle frame
        # This probably needs some work
        for i in range(idx, min(idx+100, len(self.positions)-1)):
            pose = self.tf_buffer.transform(self.global_path.poses[i], 'vehicle')
            local_path.poses.append(pose)

        # Publish local path and global path (just to refresh, even though it doesn't change)
        self.publisher_local.publish(local_path)
        self.publisher_global.publish(self.global_path)



def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)

    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

