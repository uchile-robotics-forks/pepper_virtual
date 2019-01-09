#!/usr/bin/env python
from math import atan2, cos, sin, sqrt

import rospy

from tf import TransformListener
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan, PointCloud2
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.point_cloud2 import read_points, create_cloud_xyz32


class LaserPublisher(object):
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('laser_test', anonymous=True, disable_signals=True)
            print "Initialised rospy node: laser_test"

        self.tl = TransformListener()
        self.lp = LaserProjection()

        self.all_laser_pub = rospy.Publisher(
            '/pepper/laser', LaserScan, queue_size=1)

        # Subscribers
        left_sub = Subscriber('/pepper/scan_left', LaserScan)
        front_sub = Subscriber('/pepper/scan_front', LaserScan)
        right_sub = Subscriber('/pepper/scan_right', LaserScan)

        self.ts = TimeSynchronizer([left_sub, front_sub, right_sub],
                                   10)
        self.ts.registerCallback(self.scan_cb)

        print "Finished intialising"

    def scan_cb(self, left, front, right):
        translated_points = []
        try:
            pc_left = self.lp.projectLaser(left)
            pc_front = self.lp.projectLaser(front)
            pc_right = self.lp.projectLaser(right)
        except Exception as e:
            print "Failed to transform laser scan because: " + str(e)

        # right point cloud translation
        for p in read_points(pc_right, field_names=('x', 'y', 'z'), skip_nans=True):
            x = p[0]
            y = p[1]
            # converts the position wrt the right laser frame into the position wrt the front laser frame
            newx = x * cos(-1.757) - y * sin(-1.757)
            newy = x * sin(-1.757) + y * cos(-1.757)
            point = (newx, newy, 0.0)
            translated_points.append(point)

        # front point cloud
        for p in read_points(pc_front, field_names=('x', 'y', 'z'), skip_nans=True):
            point = (p[0], p[1], p[2])
            translated_points.append(point)

        # left pc translation
        for p in read_points(pc_left, field_names=('x', 'y', 'z'), skip_nans=True):
            x = p[0]
            y = p[1]
            # converts the position wrt the left laser frame into the position wrt the front laser frame
            newx = x * cos(1.757) - y * sin(1.757)
            newy = x * sin(1.757) + y * cos(1.757)
            point = (newx, newy, 0.0)
            translated_points.append(point)

        # Create a point cloud from the combined points wrt the front laser frame
        point_cloud = create_cloud_xyz32(pc_front.header, translated_points)

        # Convert combined point cloud into LaserScan
        all_laser_msg = front
        laser_ranges, angle_min, angle_max = self.pc_to_laser(point_cloud)
        all_laser_msg.ranges = laser_ranges
        all_laser_msg.angle_min = angle_min
        all_laser_msg.angle_max = angle_max
        self.all_laser_pub.publish(all_laser_msg)

    def pc_to_laser(self, cloud):
        laser_points = []
        min_angle = None
        previous_angle = None
        angle_increment = None
        points = read_points(cloud, skip_nans=True)
        
        # Convert each point cloud point into a LaserScan
        for idx, p in enumerate(points):
            current_angle = atan2(p[1], p[0])
            if idx == 0:
                # Get the smallest angle
                min_angle = current_angle
            if previous_angle and not angle_increment:
                angle_increment = abs(current_angle - previous_angle)
            if angle_increment:
                angle = current_angle - previous_angle
                # Adds -1 where there are holes in the laser scans
                # Since pepper has 3 x 60 degree lasers with 2 x 20
                # degree gaps between front and side lasers
                if angle > angle_increment:
                    if int(angle / angle_increment) > 1:
                        for i in range(int(angle / angle_increment)):
                            laser_points.append(-1)
            laser_points.append(self.get_dist(p[0], p[1]))
            previous_angle = current_angle
        # Get the max angle
        max_angle = previous_angle
        return laser_points, min_angle, max_angle

    def get_dist(self, x0, y0, x1=0, y1=0):
        return sqrt((x1 - x0)**2 + (y1 - y0)**2)


if __name__ == "__main__":
    lp = LaserPublisher()
    rospy.spin()
