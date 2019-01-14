#!/usr/bin/env python
from math import atan2, cos, radians, sin, sqrt

import rospy

from tf import TransformListener
from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan, PointCloud2
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.point_cloud2 import read_points, create_cloud_xyz32


class LaserPublisher(object):
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('laser_test', anonymous=True, disable_signals=True)
            rospy.loginfo("Initialised rospy node: laser_test")

        self.tl = TransformListener()
        self.lp = LaserProjection()

        # Publishers
        self.all_laser_pub = rospy.Publisher(
            '/pepper/laser', LaserScan, queue_size=1)
        self.pc_pub = rospy.Publisher('/cloud', PointCloud2, queue_size=1)

        # Subscribers
        left_sub = Subscriber('/pepper/scan_left', LaserScan)
        front_sub = Subscriber('/pepper/scan_front', LaserScan)
        right_sub = Subscriber('/pepper/scan_right', LaserScan)

        self.ts = TimeSynchronizer([left_sub, front_sub, right_sub],
                                   10)
        self.ts.registerCallback(self.scan_cb)

        rospy.loginfo("Finished intialising")

    def scan_cb(self, left, front, right):
        translated_points = []
        try:
            pc_left = self.lp.projectLaser(left)
            pc_front = self.lp.projectLaser(front)
            pc_right = self.lp.projectLaser(right)
        except Exception as e:
            rospy.logerr("Failed to transform laser scan because: " + str(e))

        # right point cloud translation
        for p in read_points(pc_right,
                             field_names=('x', 'y', 'z'),
                             skip_nans=True):
            x = p[0]
            y = p[1]
            # converts the position wrt the right laser frame into the
            # position wrt the front laser frame
            newx = x * cos(-1.757) - y * sin(-1.757)
            newy = x * sin(-1.757) + y * cos(-1.757)
            point = (newx, newy, 0.0)
            translated_points.append(point)

        # front point cloud
        for p in read_points(pc_front,
                             field_names=('x', 'y', 'z'),
                             skip_nans=True):
            point = (p[0], p[1], p[2])
            translated_points.append(point)

        # left pc translation
        for p in read_points(pc_left,
                             field_names=('x', 'y', 'z'),
                             skip_nans=True):
            x = p[0]
            y = p[1]
            # converts the position wrt the left laser frame into the
            # position wrt the front laser frame
            newx = x * cos(1.757) - y * sin(1.757)
            newy = x * sin(1.757) + y * cos(1.757)
            point = (newx, newy, 0.0)
            translated_points.append(point)

        # Create a point cloud from the combined points wrt the front
        # laser frame
        point_cloud = create_cloud_xyz32(pc_front.header, translated_points)
        # self.pc_pub.publish(point_cloud)

        # Convert combined point cloud into LaserScan
        all_laser_msg = front
        laser_ranges, angle_min, angle_max, angle_increment = self.pc_to_laser(
            point_cloud)
        all_laser_msg.ranges = laser_ranges
        all_laser_msg.angle_min = angle_min
        all_laser_msg.angle_max = angle_max
        all_laser_msg.angle_increment = angle_increment
        self.all_laser_pub.publish(all_laser_msg)

    def pc_to_laser(self, cloud):
        laser_points = [-1] * 61
        min_angle = -2.28059911728
        max_angle = 2.28059896867
        angle_increment = 0.07479
        for p in read_points(cloud, skip_nans=True):
            current_angle = atan2(p[1], p[0])
            index = int(round(((current_angle - min_angle) /
                               (max_angle - min_angle)) * (len(laser_points) - 1)))
            if index > (len(laser_points) - 1):
                laser_points.append(self.get_dist(p[0], p[1]))
            else:
                laser_points[index] = self.get_dist(p[0], p[1])
        # max_angle = current_angle
        return laser_points, min_angle, max_angle, angle_increment

    def get_dist(self, x0, y0, x1=0, y1=0):
        return sqrt((x1 - x0)**2 + (y1 - y0)**2)


if __name__ == "__main__":
    lp = LaserPublisher()
    rospy.spin()
