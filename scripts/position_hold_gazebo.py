#!/usr/bin/env python

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():

    def __init__(self):

        # initializing ros node with name drone_control
        rospy.init_node('drone_control')

        self.drone_position = [0.0, 0.0, 0.0]

        self.setpoint = [2., 2., 20.]

        # Declaring a cmd of message type edrone_msgs and initializing values
        self.cmd = edrone_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        self.Kp = [36., 42.06, 47.1]
        self.Ki = [.144, .176, .2]
        self.Kd = [750., 923.7, 780.]

        # self.Kp = [45.9, 40.7, 90.2]
        # self.Ki = [0., 0., 0.]
        # self.Kd = [1090.9, 936.6, 1300.2]

        self.prev_errors = [0., 0., 0.]
        self.min_values = [1000, 1000, 1000]
        self.max_values = [2000, 2000, 2000]
        self.Iterm = [0., 0., 0.]

        self.sample_time = 0.060  # in seconds

        self.command_pub = rospy.Publisher(
            '/drone_command', edrone_msgs, queue_size=1)
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)

        self.arm()  # ARMING THE DRONE

    # Disarming condition of the drone
    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone : Best practise is to disarm and then arm the drone.
    def arm(self):

        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def roll_pid(self):
        error = self.drone_position[0] - self.setpoint[0]
        self.Iterm[0] += (error+self.prev_errors[0]) * self.Ki[0]
        output = self.Kp[0]*error + self.Kd[0] * \
            (error-self.prev_errors[0]) + self.Iterm[0]
        self.prev_errors[0] = error
        # self.roll_pub.publish(error)
        return output

    def pitch_pid(self):
        error = self.drone_position[1] - self.setpoint[1]
        self.Iterm[1] += (error+self.prev_errors[1]) * self.Ki[1]
        output = self.Kp[1]*error + self.Kd[1] * \
            (error-self.prev_errors[1]) + self.Iterm[1]
        self.prev_errors[1] = error
        # self.pitch_pub.publish(error)
        return output

    def alt_pid(self):
        error = self.drone_position[2] - self.setpoint[2]
        self.Iterm[2] += (error+self.prev_errors[2]) * self.Ki[2]
        output = self.Kp[2]*error + self.Kd[2] * \
            (error-self.prev_errors[2]) + self.Iterm[2]
        self.prev_errors[2] = error
        # self.alt_pub.publish(error)
        return output

    def pid(self):

        roll_out = self.roll_pid()
        self.cmd.rcRoll = 1500 - roll_out
        self.cmd.rcRoll = min(2000, self.cmd.rcRoll)
        self.cmd.rcRoll = max(1000, self.cmd.rcRoll)

        pitch_out = self.pitch_pid()
        self.cmd.rcPitch = 1500 + pitch_out
        self.cmd.rcPitch = min(2000, self.cmd.rcPitch)
        self.cmd.rcPitch = max(1000, self.cmd.rcPitch)

        alt_out = self.alt_pid()
        self.cmd.rcThrottle = 1500 + alt_out
        self.cmd.rcThrottle = min(2000, self.cmd.rcThrottle)
        self.cmd.rcThrottle = max(1000, self.cmd.rcThrottle)

        self.command_pub.publish(self.cmd)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()