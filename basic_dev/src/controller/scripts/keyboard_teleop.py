#!/usr/bin/env python3

import rospy
import struct
import genpy
from std_msgs.msg import Header
from airsim_ros.srv import Takeoff, Land, Reset
import sys, select, termios, tty

# Dynamically construct the correct Simulator VelCmd
# (matches the simulator's expected message structure, not the local .msg file)
class SimulatorVelCmd(genpy.Message):
    _md5sum = "1a888a503939ba521aed68feee8eab6d"
    _type = "airsim_ros/VelCmd"
    _has_header = True
    _full_text = """std_msgs/Header header
float64 vx
float64 vy
float64 vz
float64 yawRate
uint8 va
uint8 stop
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
"""
    __slots__ = ['header', 'vx', 'vy', 'vz', 'yawRate', 'va', 'stop']
    _slot_types = ['std_msgs/Header', 'float64', 'float64', 'float64', 'float64', 'uint8', 'uint8']

    def __init__(self, *args, **kwds):
        super(SimulatorVelCmd, self).__init__(*args, **kwds)
        if self.header is None:
            self.header = Header()
        if self.vx is None: self.vx = 0.
        if self.vy is None: self.vy = 0.
        if self.vz is None: self.vz = 0.
        if self.yawRate is None: self.yawRate = 0.
        if self.va is None: self.va = 0
        if self.stop is None: self.stop = 0

    def serialize(self, buff):
        self.header.serialize(buff)
        buff.write(struct.Struct("<4d2B").pack(self.vx, self.vy, self.vz, self.yawRate, self.va, self.stop))

    def deserialize(self, str):
        pass

msg = """
--------------------------------------------------
  Keyboard Teleop for AirSim Drone
--------------------------------------------------
  Movement (hold key to keep moving):

     i        : forward  (vx+)
     k        : backward (vx-)
     j        : left     (vy+)
     l        : right    (vy-)
     u / UP   : up       (vz+)
     o / DOWN : down     (vz-)
     a        : yaw CCW  (yawRate+)
     d        : yaw CW   (yawRate-)

  Commands:
     t : Takeoff
     b : Land
     r : Reset
     SPACE : Emergency stop (hover)

  CTRL-C to quit
--------------------------------------------------
"""

# (vx, vy, vz, yawRate)
moveBindings = {
    'i': ( 1,  0,  0,  0),
    'k': (-1,  0,  0,  0),
    'j': ( 0,  1,  0,  0),
    'l': ( 0, -1,  0,  0),
    'u': ( 0,  0,  1,  0),
    'o': ( 0,  0, -1,  0),
    'a': ( 0,  0,  0,  1),
    'd': ( 0,  0,  0, -1),
    '\x1b[A': ( 0, 0,  1, 0),  # Arrow UP
    '\x1b[B': ( 0, 0, -1, 0),  # Arrow DOWN
}

settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('/airsim_node/drone_1/vel_body_cmd', SimulatorVelCmd, queue_size=10)

    rospy.loginfo("Waiting for services (timeout 2s)...")
    try:
        rospy.wait_for_service('/airsim_node/drone_1/takeoff', timeout=2.0)
        takeoff_srv = rospy.ServiceProxy('/airsim_node/drone_1/takeoff', Takeoff)
        rospy.loginfo("Takeoff service found.")
    except rospy.ROSException:
        rospy.logwarn("Takeoff service not found.")
        takeoff_srv = None

    try:
        rospy.wait_for_service('/airsim_node/drone_1/land', timeout=2.0)
        land_srv = rospy.ServiceProxy('/airsim_node/drone_1/land', Land)
        rospy.loginfo("Land service found.")
    except rospy.ROSException:
        rospy.logwarn("Land service not found.")
        land_srv = None

    try:
        rospy.wait_for_service('/airsim_node/reset', timeout=2.0)
        reset_srv = rospy.ServiceProxy('/airsim_node/reset', Reset)
        rospy.loginfo("Reset service found.")
    except rospy.ROSException:
        rospy.logwarn("Reset service not found.")
        reset_srv = None

    speed = 2.0   # m/s
    turn  = 1.0   # rad/s
    accel = 4     # acceleration m/s^2, max 8

    # Persistent velocity state (holds last command until changed)
    vx = 0; vy = 0; vz = 0; yaw = 0
    stop_flag = 0

    print(msg)

    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            key = getKey()

            if key in moveBindings:
                vx  = moveBindings[key][0]
                vy  = moveBindings[key][1]
                vz  = moveBindings[key][2]
                yaw = moveBindings[key][3]
                stop_flag = 0
            elif key == 't' and takeoff_srv:
                rospy.loginfo(">> Takeoff")
                try: takeoff_srv(waitOnLastTask=True)
                except Exception as e: rospy.logwarn(f"Takeoff failed: {e}")
                continue
            elif key == 'b' and land_srv:
                rospy.loginfo(">> Land")
                try: land_srv(waitOnLastTask=True)
                except Exception as e: rospy.logwarn(f"Land failed: {e}")
                continue
            elif key == 'r' and reset_srv:
                rospy.loginfo(">> Reset")
                try: reset_srv()
                except Exception as e: rospy.logwarn(f"Reset failed: {e}")
                continue
            elif key == ' ':
                # Emergency stop
                vx = 0; vy = 0; vz = 0; yaw = 0
                stop_flag = 1
            elif key == '':
                # No key pressed (timeout), keep sending LAST velocity
                pass
            else:
                if key == '\x03':  # CTRL-C
                    break
                # Unknown key => stop
                vx = 0; vy = 0; vz = 0; yaw = 0
                stop_flag = 0

            cmd = SimulatorVelCmd()
            cmd.header.stamp = rospy.Time.now()
            cmd.vx      = float(vx * speed)
            cmd.vy      = float(vy * speed)
            cmd.vz      = float(vz * speed)
            cmd.yawRate  = float(yaw * turn)
            cmd.va      = 0 if stop_flag else accel
            cmd.stop    = stop_flag

            pub.publish(cmd)
            rate.sleep()

    except Exception as e:
        print(e)
    finally:
        # Send stop command on exit
        cmd = SimulatorVelCmd()
        cmd.header.stamp = rospy.Time.now()
        cmd.stop = 1
        cmd.va = 0
        pub.publish(cmd)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    main()
