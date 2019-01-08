import rospy
from message_filters import TimeSynchronizer, Subscriber
from math import sqrt, cos, sin
from sensor_msgs.msg import LaserScan, PointCloud2
from tf import TransformListener
from geometry_msgs.msg import PointStamped
from laser_geometry import LaserProjection
from sensor_msgs.point_cloud2 import read_points, create_cloud_xyz32


class LaserMerger(object):
    def __init__(self):
        rospy.init_node('laser_merger', anonymous=True, disable_signals=True)

        self.tl = TransformListener()
        self.lp = LaserProjection()

        # Subscribers
        left_sub = Subscriber('/pepper/scan_left', LaserScan)
        front_sub = Subscriber('/pepper/scan_front', LaserScan)
        right_sub = Subscriber('/pepper/scan_right', LaserScan)

        # Publishers
        self.left_pub = rospy.Publisher(
            '/pepper/pc_left', PointCloud2, queue_size=1)
        self.front_pub = rospy.Publisher(
            '/pepper/pc_front', PointCloud2, queue_size=1)
        self.right_pub = rospy.Publisher(
            '/pepper/pc_right', PointCloud2, queue_size=1)
        self.cloud_pub = rospy.Publisher(
            'cloud_in', PointCloud2, queue_size=1)

        self.ts = TimeSynchronizer(
            [left_sub, front_sub, right_sub], 10)
        self.ts.registerCallback(self.scan_cb)

        # For debugging
        self.a = 0

    def get_dist(self, x0, y0, x1=0, y1=0):
        return sqrt((x1 - x0)**2 + (y1 - y0)**2)

    def scan_cb(self, left, front, right):
        try:
            pc_left = self.lp.projectLaser(left)
            pc_front = self.lp.projectLaser(front)
            pc_right = self.lp.projectLaser(right)
        except Exception as e:
            print "Failed to transform laser scan because: " + str(e)

        # For debugging
        nontranslated_points = []
        translated_points = []

        nontranslated_points.append("Data set " + str(self.a))
        self.a += 1

        # left pc translation
        nontranslated_points.append("Left")
        for p in read_points(pc_left, skip_nans=True):
            nontranslated_points.append(p)
            x = p[0]
            y = p[1]
            newx = x * cos(1.757) - y * sin(1.757)  # - 0.102
            newy = x * sin(1.757) + y * cos(1.757)  # + 0.056
            point = (newx, newy, 0.0)
            translated_points.append(point)

        nontranslated_points.append("Front")
        # front point cloud
        for p in read_points(pc_front, skip_nans=True):
            nontranslated_points.append(p)
            point = (p[0], p[1], p[2])
            translated_points.append(point)

        # right point cloud translation
        nontranslated_points.append("Right")
        for p in read_points(pc_right, skip_nans=True):
            nontranslated_points.append(p)
            x = p[0]
            y = p[1]
            newx = x * cos(-1.757) - y * sin(-1.757)  # - 0.102
            newy = x * sin(-1.757) + y * cos(-1.757)  # - 0.056
            point = (newx, newy, 0.0)
            translated_points.append(point)

        print translated_points
        point_cloud = create_cloud_xyz32(pc_front.header, translated_points)

        self.left_pub.publish(pc_left)
        self.front_pub.publish(pc_front)
        self.right_pub.publish(pc_right)
        self.cloud_pub.publish(point_cloud)


if __name__ == "__main__":
    lm = LaserMerger()
    rospy.spin()
