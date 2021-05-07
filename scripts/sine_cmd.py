#!/usr/bin/env python

import rospy
from math import sin
from franka_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class Spinner:
    def __init__(self, sine_freq_rad=0.1, sine_freq_amplitude=1):
        self.sine_freq_rad = sine_freq_rad
        self.sine_freq_amplitude = sine_freq_amplitude
        
        self.init_time = rospy.get_time()
        self.prev_time = rospy.get_time()
        self.init_qd = []

        self.q_now = {"panda_joint1": 0.0, "panda_joint2": 0.0, "panda_joint3": 0.0, "panda_joint4": 0.0, "panda_joint5": 0.0, "panda_joint6": 0.0, "panda_joint7": 0.0}
        self.qd_now = {"panda_joint1": 0.0, "panda_joint2": 0.0, "panda_joint3": 0.0, "panda_joint4": 0.0, "panda_joint5": 0.0, "panda_joint6": 0.0, "panda_joint7": 0.0}

        self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

        self.sub_joint_states = rospy.Subscriber("/joint_states", JointState, self._callback_JointState)
        self.sub_joint_states_desired = rospy.Subscriber("/joint_states_desired", JointState, self._callback_JointStateDesired)
        self.pub_joint_impedance_cmd = rospy.Publisher("/franka_ros_interface/motion_controller/arm/joint_commands", JointCommand, tcp_nodelay=True, queue_size=1)
        self.pub_test_debug = rospy.Publisher("/test_debug", Float64, queue_size=1)

        self.joint_position_upper_limits = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973,	3.7525,	2.8973]
        self.joint_position_lower_limits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]

    def _update_cmd(self, t):
        return self.sine_freq_amplitude*(sin(self.sine_freq_rad*t))

    def _callback_JointState(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self.joint_names:
                self.qd_now[name] = msg.position[idx]

    def _callback_JointStateDesired(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self.joint_names:
                self.qd_now[name] = msg.position[idx]

        if not self.init_qd:
            self.init_qd = self.qd_now

        sec = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        t = sec - self.init_time

        #self._publish_test_cmd(t)
        self._publish_joint_impedance_cmd(t)

        tmp = 1e6*(0.001 - (t-self.prev_time))
        self.pub_test_debug.publish(tmp)

        self.prev_time = t


    def _publish_joint_impedance_cmd(self, t):
        joint_num = 'panda_joint4'
        qd = self.init_qd
        qd[joint_num] = self.init_qd[joint_num]+ self._update_cmd(t)
        #qd[joint_num] = min(max(0.9*self.joint_position_lower_limits[joint_num], qd[joint_num]), 0.9*self.joint_position_upper_limits[joint_num])

        msg = JointCommand()
        msg.names = self.joint_names
        msg.position = [qd[j] for j in self.joint_names]
        msg.velocity = [0.0 for _ in range(7)]
        msg.mode = JointCommand.IMPEDANCE_MODE
        msg.header.stamp = rospy.Time.now()

        self.pub_joint_impedance_cmd.publish(msg)


if __name__ == '__main__':
    rospy.init_node("test_node")
    Spinner(sine_freq_rad=0.2, sine_freq_amplitude=0.5)
    rospy.loginfo("Initialized.")
    rospy.spin()
