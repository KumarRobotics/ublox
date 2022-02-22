#!/usr/bin/python3
import rospy
from ublox_msgs.msg import NavHPPOSLLH, NavRELPOSNED9, NavVELNED

class Talker(object):
    def __init__(self):
        self.itow = 0
        self.pub1 = rospy.Publisher('msg1', NavHPPOSLLH, queue_size=2)
        self.pub2 = rospy.Publisher('msg2', NavRELPOSNED9, queue_size=2)
        self.pub3 = rospy.Publisher('msg3', NavVELNED, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(0.2),self.publish)

    def publish(self, event):
        rospy.loginfo("TX %u", self.itow)

        msg1 = NavHPPOSLLH()
        msg1.iTOW = self.itow
        self.pub1.publish(msg1)

        msg2 = NavRELPOSNED9()
        msg2.iTOW = self.itow
        self.pub2.publish(msg2)

        msg3 = NavVELNED()
        msg3.iTOW = self.itow
        self.pub3.publish(msg3)

        self.itow += 1

def main():
    rospy.init_node('talker')
    talker = Talker()
    rospy.loginfo("Ready to transmit")
    rospy.spin()

if __name__ == '__main__':
    main()
