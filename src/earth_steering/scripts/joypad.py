#!/usr/bin/env python
import os
import subprocess
from threading import Lock
import rospy
import rospkg
from sensor_msgs.msg import NavSatFix

from sensor_msgs.msg import Joy
from four_wheel_steering_msgs.msg import FourWheelSteering

import navigation_api

INPLACE_FILE = "/home/earth/catkin_ws/scripts/rotate_in_place.sh"
# WAYPOINT_FILE = "/home/earth/catkin_ws/earth-rover-ros/earth_rover_navigation/src/client/geo_wp.txt"

mutex = Lock()

node = None

def fix_callback(data):
    mutex.acquire()
    try:
        node.state["fix"] = {
            "status": data.status.status,
            "service": data.status.service,
            "lat": data.latitude,
            "lon": data.longitude,
            "alt": data.altitude,
        }
    finally:
        mutex.release()


class SteeringTransformNode(object):


    def __init__(self):
        self.last_sec = 0
        self.last_nano = 0
        self.next_sec = None
        self.next_nano = None
        self.every_nanos = 100000000
        self.button_states = 19 * [0]
        self.button_handlers = 19 * [None]
        self.state = {}
        self.inplace = False
        self.mode = "auto"


        self.velocity_scale = 0.5
        self.crab_scale = 0.7
        self.turn_scale = 0.7
        self.velocity_boost = 3
        self.max_angle = 0.7
        self.angle_change_rate = 0.5
        self.acceleration = 0.5
        self.jerk = 0.5

        self.button_handlers[3] = self.nav_start
        self.button_handlers[4] = self.toggle_mode
        self.button_handlers[5] = self.run_command_5
        self.button_handlers[6] = self.run_command_6
        self.button_handlers[7] = self.run_command_7
        self.button_handlers[13] = self.nav_clear
        self.button_handlers[14] = self.nav_reset
        # self.button_handlers[15] = self.nav_pause
        self.button_handlers[12] = self.nav_add
        self.button_handlers[16] = self.run_in_place

        rospack = rospkg.RosPack()
        dir = rospack.get_path('earth_rover_navigation')
        self.nav_file_path = os.path.join(dir, "src", "client", "geo_wp.txt")

        topic = "/four_wheel_steering_controller/cmd_four_wheel_steering"
        message_type = FourWheelSteering

        self.publisher = rospy.Publisher(topic, message_type, queue_size=10)
        navigation_api.init_api()


    def time_delta(self, sec, nano):
        sec_delta = (sec - self.last_sec) * 1000000000
        nano_delta = nano - self.last_nano
        return sec_delta + nano_delta

    # def run_command_4(self):
    #     cmd = "/home/earth/catkin_ws/scripts/joy_cmd_1.sh"
    #     subprocess.Popen(cmd, shell=True)

    def run_command_5(self):

        cmd = "/home/earth/catkin_ws/scripts/joy_cmd_2.sh"
        subprocess.Popen(cmd, shell=True)

    def run_command_6(self):
        cmd = "/home/earth/catkin_ws/scripts/joy_cmd_3.sh"
        subprocess.Popen(cmd, shell=True)

    def run_command_7(self):
        cmd = "/home/earth/catkin_ws/scripts/joy_cmd_4.sh"
        subprocess.Popen(cmd, shell=True)

    def run_in_place(self):
        self.inplace = not os.path.exists(INPLACE_FILE)
        cmd = INPLACE_FILE
        subprocess.Popen(cmd, shell=True)

    def toggle_mode(self):
        if self.mode == "auto":
            self.set_mode_joypad()
        else:
            self.set_mode_auto()

    def set_mode_joypad(self):
        self.mode = "joypad"
        rospy.set_param("/steering_mode", "joypad")

    def set_mode_auto(self):
        self.mode = "auto"
        rospy.set_param("/steering_mode", "auto")


    def nav_start(self):
        self.set_mode_auto()
        self.send_waypoints()
        navigation_api.start()
    #
    # def nav_pause(self):
    #     navigation_api.pause()

    def nav_clear(self):
        self.set_mode_joypad()
        navigation_api.pause()
        navigation_api.cancel()

    def nav_reset(self):
        self.set_mode_joypad()
        navigation_api.pause()
        navigation_api.cancel()
        if os.path.exists(self.nav_file_path):
            os.remove(self.nav_file_path)
        with open(self.nav_file_path, 'w') as f:
            f.write("Type: geo\n")


    def send_waypoints(self):
        cmd = "/home/earth/catkin_ws/scripts/send_waypoints.sh"
        subprocess.call(cmd, shell=True)

        #
        # if not os.path.exists(self.nav_file_path):
        #     return
        #
        # cartesian = False
        #
        # with open(self.nav_file_path, 'r') as f:
        #     for line in f.readlines():
        #         if "Type" in line:
        #             if "cartesian" in line:
        #                 cartesian = True
        #         else:
        #             x, y, a = line.split(" ")
        #             if cartesian:
        #                 navigation_api.add_waypoint_cartesian(float(x), float(y), float(a))
        #             else:
        #                 navigation_api.add_waypoint(float(x), float(y), float(a))

    def nav_add(self):

        if not os.path.exists(self.nav_file_path):
            with open(self.nav_file_path, 'a') as f:
                f.write("Type: geo\n")

        mutex.acquire()
        fix = self.state.get("fix")
        if fix is not None:
            fix = fix.copy()
        mutex.release()

        if fix is not None:
            with open(self.nav_file_path, 'a') as f:
                x = fix["lat"]
                y = fix["lon"]
                bearing = 0
                f.write("%s %s %s\n" % (x, y, bearing))



    def get_velocity(self, joy):

        velocity = -(joy.axes[13] - 1) * float(self.velocity_scale)
        if velocity == 0.5:
            velocity = 0.0

        if joy.axes[12] != 0 and joy.axes[12] < 1 :
            velocity = (joy.axes[12] - 1) * float(self.velocity_scale)
#        rospy.loginfo("velocity: %s" % velocity)
        return velocity


    def get_crab(self, joy):

        crab = joy.axes[0]/float(self.crab_scale)
        if self.inplace:
            return 0.0
        else:
            return crab


    def get_turn(self, joy):
        turn = joy.axes[2]/float(self.turn_scale)
        if self.inplace:
            return 0.0
        else:
            return turn


    def get_angles_scaled(self, joy, scale):

        crab = self.get_crab(joy) * scale
        turn = self.get_turn(joy) * scale
        front = turn/2.0 + crab
        back = -turn/2.0 + crab
        return front, back


    def get_angles(self, joy):

        front, back = self.get_angles_scaled(joy, 1.0)
        abs_front = abs(front)
        abs_back = abs(back)
        abs_max = max(abs_front, abs_back)
        if abs_max > self.max_angle:
            front, back = self.get_angles_scaled(joy, self.max_angle/abs_max)

        return front, back


    def send_message(self, velocity, front, back):

        msg = FourWheelSteering(front_steering_angle=front,
            rear_steering_angle=back,
            front_steering_angle_velocity=self.angle_change_rate,
            rear_steering_angle_velocity=self.angle_change_rate,
            speed=velocity,
            acceleration=self.acceleration,
            jerk=self.jerk)

        self.publisher.publish(msg)


    def check_buttons(self, joy):
        for i, button_state in enumerate(joy.buttons):

            if button_state == 1 and self.button_states[i] == 0:
                handler = self.button_handlers[i]
                print("pressed button %s" % i)
                if handler is not None:
                    handler()
            self.button_states[i] = button_state


    def __call__(self, joy):

        self.check_buttons(joy)

        if self.mode == "joypad":
            velocity = self.get_velocity(joy)
            front, back = self.get_angles(joy)
            self.send_message(velocity, front, back)


def listener():
    global node
    node = SteeringTransformNode()

    rospy.init_node('earth_joypad_steering', anonymous=True)
    rospy.Subscriber("joy", Joy, node)
    rospy.Subscriber("/earth_gps/fix", NavSatFix, fix_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
