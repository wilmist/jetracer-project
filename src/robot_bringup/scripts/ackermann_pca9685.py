#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, time
from ackermann_msgs.msg import AckermannDriveStamped

# Adafruit PCA9685 uses Python2 package Adafruit_PCA9685
import Adafruit_PCA9685

def clamp(x, a, b):
    return max(a, min(b, x))

class PcaAckermann(object):
    def __init__(self):
        # Params (tune these!)
        self.i2c_bus = rospy.get_param("~i2c_bus", 1)
        self.i2c_addr = rospy.get_param("~i2c_addr", 0x40)

        # Channels
        self.steer_ch = int(rospy.get_param("~steer_channel", 0))
        self.throttle_ch = int(rospy.get_param("~throttle_channel", 1))

        # Steering (radians -> PWM)
        self.steer_max_angle = float(rospy.get_param("~steer_max_angle", 0.5))  # ~28.6°
        self.steer_center_us = int(rospy.get_param("~steer_center_us", 1500))
        self.steer_left_us   = int(rospy.get_param("~steer_left_us",   1900))   # +max angle
        self.steer_right_us  = int(rospy.get_param("~steer_right_us",  1100))   # -max angle

        # Throttle (ESC PWM)
        self.speed_max = float(rospy.get_param("~speed_max", 2.0))  # m/s corresponding to ~full forward
        self.throttle_neutral_us = int(rospy.get_param("~throttle_neutral_us", 1500))
        self.throttle_min_us     = int(rospy.get_param("~throttle_min_us",     1100))  # full reverse (if enabled)
        self.throttle_max_us     = int(rospy.get_param("~throttle_max_us",     1900))  # full forward

        self.deadman_timeout = float(rospy.get_param("~deadman_timeout", 0.5))  # sec without cmd -> neutral
        self.last_cmd_time = rospy.Time.now()

        # Setup PCA9685
        self.pwm = Adafruit_PCA9685.PCA9685(address=self.i2c_addr, busnum=self.i2c_bus)
        self.pwm.set_pwm_freq(50)  # 50Hz for servo/ESC

        # Center on startup
        self.write_us(self.steer_ch, self.steer_center_us)
        self.write_us(self.throttle_ch, self.throttle_neutral_us)

        rospy.Subscriber("~cmd", AckermannDriveStamped, self.cmd_cb, queue_size=5)
        # Also listen to /ackermann_cmd by default
        rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.cmd_cb, queue_size=5)

        # Deadman timer
        self.timer = rospy.Timer(rospy.Duration(0.05), self.deadman_cb)

    def us_to_pca(self, usec):
        # PCA9685 is 12-bit (0..4095), 20ms period at 50Hz
        # pulse width fraction = usec / 20000
        ticks = int(4096.0 * (usec / 20000.0))
        return clamp(ticks, 0, 4095)

    def write_us(self, ch, usec):
        ticks = self.us_to_pca(usec)
        self.pwm.set_pwm(ch, 0, ticks)

    def angle_to_us(self, angle):
        # Map [-steer_max, +steer_max] -> [right_us, left_us], center at 0
        a = clamp(angle / self.steer_max_angle, -1.0, 1.0)
        if a >= 0.0:
            return int(self.steer_center_us + a * (self.steer_left_us - self.steer_center_us))
        else:
            return int(self.steer_center_us + a * (self.steer_center_us - self.steer_right_us))

    def speed_to_us(self, speed):
        # Map [-speed_max, +speed_max] -> [min_us, max_us], neutral at 0
        s = clamp(speed / self.speed_max, -1.0, 1.0)
        if s > 0.0:
            return int(self.throttle_neutral_us + s * (self.throttle_max_us - self.throttle_neutral_us))
        elif s < 0.0:
            return int(self.throttle_neutral_us + s * (self.throttle_neutral_us - self.throttle_min_us))
        else:
            return self.throttle_neutral_us

    def cmd_cb(self, msg):
        self.last_cmd_time = rospy.Time.now()
        steer_us = self.angle_to_us(msg.drive.steering_angle)
        throttle_us = self.speed_to_us(msg.drive.speed)
        self.write_us(self.steer_ch, steer_us)
        self.write_us(self.throttle_ch, throttle_us)

    def deadman_cb(self, _evt):
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.deadman_timeout:
            # neutral/center
            self.write_us(self.steer_ch, self.steer_center_us)
            self.write_us(self.throttle_ch, self.throttle_neutral_us)

if __name__ == "__main__":
    rospy.init_node("ackermann_pca9685")
    PcaAckermann()
    rospy.spin()

