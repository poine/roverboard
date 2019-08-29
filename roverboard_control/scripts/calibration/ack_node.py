#!/usr/bin/env python
import sys, time, numpy as np
import rospy, ackermann_msgs.msg

import two_d_guidance.trr.rospy_utils as trr_rpu

class Node(trr_rpu.PeriodicNode):
    def __init__(self, ctl):
        trr_rpu.PeriodicNode.__init__(self, 'ack_drive_line')
        self.ctl = ctl
        cmd_topic = rospy.get_param('~cmd_topic', '/oscar_ackermann_controller/cmd_ack')
        self.pub = rospy.Publisher(cmd_topic, ackermann_msgs.msg.AckermannDriveStamped, queue_size=1)
        odom_topic = '/oscar_ackermann_controller/odom'
        self.odom_sub = trr_rpu.OdomListener(odom_topic, 'odom calib')
       
    def publish_ack(self, alpha, vel):
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.drive.steering_angle, msg.drive.speed = alpha, vel
        self.pub.publish(msg)

    def periodic(self):
        try:
            p, psi = self.odom_sub.get_loc_and_yaw()
            if not self.ctl.initialized():
                self.ctl.initialize(p, psi)
            else:
                self.publish_ack(*self.ctl.get(p, psi))
                if self.ctl.finished(p, psi): self.finish()
        except trr_rpu.NoRXMsgException, trr_rpu.RXMsgTimeoutException:
            rospy.loginfo_throttle(1., "no odom")
        
    def finish(self):
        time.sleep(0.5)
        p, psi = self.odom_sub.get_loc_and_yaw()
        print('{}'.format(self.ctl._dist(p)))
        rospy.signal_shutdown('done')
