#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
# Replace this with your actual PCA9685 or motor driver library
class Driver(object):
    def __init__(self, dev, steer_ch, throttle_ch):
        # init hardware
        pass
    def set_steer(self, norm):  # -1..1
        pass
    def set_throttle(self, norm):  # -1..1
        pass

class Node(object):
    def __init__(self):
        self.steer_max = rospy.get_param("~steer_max_angle", 0.5)
        self.speed_max = rospy.get_param("~speed_max", 2.0)
        dev = rospy.get_param("~pwm_device", "/dev/i2c-1")
        steer_ch = rospy.get_param("~steer_channel", 0)
        throttle_ch = rospy.get_param("~throttle_channel", 1)
        self.hw = Driver(dev, steer_ch, throttle_ch)
        rospy.Subscriber(rospy.get_param("~steer_topic","/ackermann_cmd"),
                         AckermannDriveStamped, self.cb, queue_size=5)
    def cb(self, msg):
        steer = max(min(msg.drive.steering_angle / self.steer_max, 1.0), -1.0)
        speed = max(min(msg.drive.speed / self.speed_max, 1.0), -1.0)
        self.hw.set_steer(steer)
        self.hw.set_throttle(speed)
    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("ackermann_to_pwm")
    Node().spin()

