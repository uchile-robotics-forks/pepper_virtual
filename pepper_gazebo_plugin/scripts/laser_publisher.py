#!/usr/bin/env python
from math import atan2, sqrt, radians

import rospy

from tf import TransformListener
from sensor_msgs.msg import LaserScan
from sensor_msgs.point_cloud2 import read_points
from message_filters import TimeSynchronizer, Subscriber
from laser_geometry.laser_geometry import LaserProjection


class LaserPublisher(object):
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('laser_test', anonymous=True, disable_signals=True)
            print "Initialised rospy node: laser_test"

        self.lp = LaserProjection()
        self.tl = TransformListener()

        self.left_laser_scans = []
        self.right_laser_scans = []
        self.front_laser_scans = []

        left_sub = Subscriber('/pepper/scan_left', LaserScan)
        front_sub = Subscriber('/pepper/scan_front', LaserScan)
        right_sub = Subscriber('/pepper/scan_right', LaserScan)
        self.ts = TimeSynchronizer([left_sub, front_sub, right_sub],
                                   10)
        self.ts.registerCallback(self.publish_all_lasers)

        self.all_laser_pub = rospy.Publisher(
            '/pepper/laser', LaserScan, queue_size=1)
        print "Finished intialising"

    # def laser_scan_cb(self, left, front, right):
    #     self.left_laser_scans.append(left)
    #     self.right_laser_scans.append(right)
    #     self.front_laser_scans.append(front)

    def publish_all_lasers(self, left, front, right):
        all_lasers = []
        all_laser_msg = front
        all_laser_msg.angle_max = left.angle_increment * (3 * 15 + 2 * 7) / 2
        all_laser_msg.angle_min = -all_laser_msg.angle_max
        for a in right.ranges:
            all_lasers.append(a)
        for a in range(8):
            all_lasers.append(-1.0)
        for a in front.ranges:
            all_lasers.append(a)
        for a in range(8):
            all_lasers.append(-1.0)
        for a in left.ranges:
            all_lasers.append(a)
        all_laser_msg.ranges = all_lasers
        all_laser_msg.intensities = []
        # all_laser_msg = self.get_laser_readings(left, right, front)
        self.all_laser_pub.publish(all_laser_msg)

    def get_dist(self, x0, y0, x1=0, y1=0):
        return sqrt((x1 - x0)**2 + (y1 - y0)**2)

    # def get_laser_readings(self, left, front, right):
    #     point_clouds = []

    #     point_clouds.append(self.lp.projectLaser(left))
    #     point_clouds.append(self.lp.projectLaser(front))
    #     point_clouds.append(self.lp.projectLaser(right))

    #     current_dist = None

    #     for pointcloud in point_clouds:
    #         for p in read_points(pointcloud2, field_names=('x', 'y', 'z'), skip_nans=True):
    #             if p[0] != 1:
    #                 distance = self.get_dist(p[0], p[1])
    #                     if not current_dist or distance < current_dist:
    #                         current_dist = distance
    #                         angle = atan2(p[1], p[0])
    #                         if angle < radians(-30):
    #                             sensor = 'right'
    #                         elif angle >= radians(-30) and angle <= radians(30):
    #                             self.get_sonar_readings()
    #                             if self.front_sonar < self.front_object_distance:
    #                                 sensor = 'front'
    #                                 self.front_obstacle = [
    #                                     sensor, p[0], p[1], p[2], current_dist, angle]
    #                         else:
    #                             sensor = 'left'
    #                         if current_dist <= self.safety_distance:
    #                             self.current_obstacle = self.front_obstacle
    #     return all_laser_msg

    def run(self):
        while not rospy.is_shutdown():
            try:
                # print "Waiting for laser scans"
                while len(self.left_laser_scans) == 0 or len(self.right_laser_scans) == 0 or len(self.front_laser_scans) == 0:
                    rospy.sleep(0.1)
                # print "Got laser scans"
                left_laser = self.left_laser_scans.pop(0)
                right_laser = self.right_laser_scans.pop(0)
                front_laser = self.front_laser_scans.pop(0)
                last_time = max([left_laser.header.stamp, right_laser.header.stamp,
                                 front_laser.header.stamp])
                # print "comparing laser scans"
                while abs(left_laser.header.stamp - last_time) > rospy.Duration(0.1) and abs(right_laser.header.stamp - last_time) > rospy.Duration(0.1) and abs(front_laser.header.stamp - last_time) > rospy.Duration(0.1) and abs(all_laser.header.stamp - last_time) > rospy.Duration(0.1):
                    if left_laser.header.stamp < last_time:
                        left_laser = self.left_laser_scans.pop(0)
                    if right_laser.header.stamp < last_time:
                        right_laser = self.right_laser_scans.pop(0)
                    if front_laser.header.stamp < last_time:
                        front_laser = self.front_laser_scans.pop(0)

                # print "Results"
                # print "======================== left laser ======================="
                # print left_laser
                # print "======================== right laser ======================"
                # print right_laser
                # print "======================== front laser ======================"
                # print front_laser
                # print "======================== all lasers ======================="
                # print all_laser
                self.publish_all_lasers(left_laser, front_laser, right_laser)
            except Exception as e:
                print ("There was an exception %s", e)


if __name__ == "__main__":
    lp = LaserPublisher()
    lp.run()
