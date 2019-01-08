import rospy
from sensor_msgs.msg import LaserScan
from message_filters import TimeSynchronizer, Subscriber


class PCLChecker(object):
    def __init__(self):
        rospy.init_node('pcl_checker', anonymous=True, disable_signals=True)
        lleft_sub = Subscriber('/pepper/scan_left', LaserScan)
        lfront_sub = Subscriber('/pepper/scan_front', LaserScan)
        lright_sub = Subscriber('/pepper/scan_right', LaserScan)
        pcleft_sub = Subscriber('/pepper/scan_left', LaserScan)
        pcfront_sub = Subscriber('/pepper/scan_front', LaserScan)
        pcright_sub = Subscriber('/pepper/scan_right', LaserScan)
        self.ts = TimeSynchronizer(
            [lleft_sub, lfront_sub, lright_sub,
             pcleft_sub, pcfront_sub, pcright_sub], 10)
        self.ts.registerCallback(self.check_values)

    def check_values(self, lleft, lfront, lright, pcleft, pcfront, pcright):


if __name__ == "__main__":
    PCLC = PCLChecker()
