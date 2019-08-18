#!/usr/bin/env python
import sys, numpy as np
import rospy, sensor_msgs.msg, visualization_msgs.msg, geometry_msgs.msg
import cv2, cv_bridge

import smocap, smocap.rospy_utils
import two_d_guidance.trr.rospy_utils as trr_rpu
import two_d_guidance.ros_utils as tdg_ru

import pdb


# class LinCalib:
#     def __init__(self):
#         self.p0 = None
#     def started(self): return self.p0 is not None
#     def start(self, p0):
#         self.p0 = p0
#     def run(self, p, d=1., v=0.1):
#         dep = p - self.p0; dist = np.linalg.norm(dep)
#         return (v, 0., dist) if  dist < d else (None, None, dist)
    
class VisionRef:
    def __init__(self):
        vision_topic = '/trr_vision/start_finish/status'
        self.sf_sub = trr_rpu.TrrStartFinishSubscriber(vision_topic, 'odom calib')

    def init(self):
        _, _, _, dist_to_finish = self.sf_sub.get()
        self.r0 = dist_to_finish
        print 'ref init'
        
    def get(self):
        _, _, _, dist_to_finish = self.sf_sub.get()
        print 'ref get'
        return self.r0 - dist_to_finish
     

class GazeboRef:
    def __init__(self):
        gz_truth_topic = '/caroline/base_link_truth1'
        self.robot_listener = tdg_ru.GazeboTruthListener(topic=gz_truth_topic)
        
    def init(self):
        p0, psi0 = self.robot_listener.get_loc_and_yaw()
        self.p0 = p0

    def get(self):
        p, psi = self.robot_listener.get_loc_and_yaw()
        return np.linalg.norm(p-self.p0)
            
class Node(trr_rpu.PeriodicNode):
    def __init__(self, calib_source, ref=None):
       trr_rpu.PeriodicNode.__init__(self, calib_source.name)
       cmd_topic = '/caroline/diff_drive_controller/cmd_vel'
       self.cmd_pub = rospy.Publisher(cmd_topic, geometry_msgs.msg.Twist, queue_size=1)
       odom_topic = '/caroline/diff_drive_controller/odom'
       self.odom_sub = trr_rpu.OdomListener(odom_topic, 'odom calib')
       self.ref = GazeboRef()
       #self.ref = VisionRef()
       self.calib = calib_source
        
    def publish_cmd(self, lin, ang):
        msg = geometry_msgs.msg.Twist()
        msg.linear.x, msg.angular.z = lin, ang
        self.cmd_pub.publish(msg)
        
    def periodic(self):
        try:
            p, psi = self.odom_sub.get_loc_and_yaw()
            if not self.calib.started():
                self.ref.init()
                self.calib.start(p, psi)
            else:
                lin, ang, dist =  self.calib.run(p, psi)
                ref = self.ref.get()
                print('{:.2f} {:.2f}'.format(dist, ref))
                self.publish_cmd(lin, ang)
                if self.calib.finished():
                    rospy.signal_shutdown('done')
        except trr_rpu.NoRXMsgException, trr_rpu.RXMsgTimeoutException:
            rospy.loginfo_throttle(1., "no odom")
        except tdg_ru.RobotNotLocalizedException:
            rospy.loginfo_throttle(1., "no reference")
        
 
def main(args):
  rospy.init_node('odom_calib_node')
  #rospy.loginfo('  using opencv version {}'.format(cv2.__version__))
  Node().run(50.)

if __name__ == '__main__':
    main(sys.argv)
