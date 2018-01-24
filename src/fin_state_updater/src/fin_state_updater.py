#!/usr/bin/python
import rospy
import roslib

import math 
import numpy

# Messages
from std_msgs.msg import Float32
from std_msgs.msg import UInt64

# Query fin robot for left and right wheel encoders.
# Publish the estimated left and right angular wheel velocities
class WheelEncoderPublisher:
  def __init__(self):
    rospy.init_node('fin_state_updater')
    # Read in tangential velocity targets
    self.lwheel_angular_vel_motor_sub = rospy.Subscriber('robot/lwheel_angular_vel_motor', Float32, self.lwheel_angular_vel_motor_callback)
    self.rwheel_angular_vel_motor_sub = rospy.Subscriber('robot/rwheel_angular_vel_motor', Float32, self.rwheel_angular_vel_motor_callback)

	# for the realy bad hack
    self.lwheel_angular_vel_control_pub = rospy.Subscriber('robot/lwheel_angular_vel_control', Float32, self.lwheel_angular_vel_control_callback)
    self.rwheel_angular_vel_control_pub = rospy.Subscriber('robot/rwheel_angular_vel_control', Float32, self.rwheel_angular_vel_control_callback)

    self.wheel_encoderL_pub = rospy.Subscriber('robot/inkL', UInt64, self.wheel_encoderL_callback)
    self.wheel_encoderR_pub = rospy.Subscriber('robot/inkR', UInt64, self.wheel_encoderR_callback)

    self.lwheel_angular_vel_enc_pub = rospy.Publisher('robot/lwheel_angular_vel_enc', Float32, queue_size=10)
    self.rwheel_angular_vel_enc_pub = rospy.Publisher('robot/rwheel_angular_vel_enc', Float32, queue_size=10)

    self.rate = rospy.get_param('~rate', 10)
    self.err_tick_incr = rospy.get_param('~err_tick_incr',20) # Filter out clearly erroneous encoder readings
    self.time_prev_update = rospy.Time.now();
    self.fin_on = rospy.get_param('~fin_on',True)
    self.leftCtr = 0
    self.rightCtr = 0
    if self.fin_on:
      #import fin   
      #self.lwheel_encslwheel_encs = [fin.enc_read(1)]*5
      #self.rwheel_encs = [fin.enc_read(0)]*5
      self.lwheel_encs = [self.leftCtr]*5
      self.rwheel_encs = [self.rightCtr]*5
    self.R = rospy.get_param('~robot_wheel_radius', .03)

    # Need a little hack to incorporate direction wheels are spinning
    self.lwheel_dir = 1;
    self.rwheel_dir = 1;
    self.rwheel_angular_vel_control = 0;
    self.lwheel_angular_vel_control = 0;
    

  def wheel_encoderL_callback(self, msg):
    self.leftCtr = msg.data

  def wheel_encoderR_callback(self, msg):
    self.rightCtr = msg.data

  # Really bad hack to get motor spin direction
  def lwheel_angular_vel_motor_callback(self,msg):
    if msg.data >= 0: self.lwheel_dir = 1
    else: self.lwheel_dir = -1
  # Really bad hack to get motor spin direction
  def rwheel_angular_vel_motor_callback(self,msg):
    if msg.data >= 0: self.rwheel_dir = 1
    else: self.rwheel_dir = -1
  def lwheel_angular_vel_control_callback(self,msg):
    self.lwheel_angular_vel_control = msg.data
  def rwheel_angular_vel_control_callback(self,msg):
    self.rwheel_angular_vel_control = msg.data

  def enc_2_rads(self,enc_cms):
    prop_revolution = (enc_cms) / (2.0*math.pi*self.R)
    rads =  prop_revolution * (2*math.pi)
    return rads

  def update(self):
    if self.fin_on: # Running on actual robot
      #import fin
      lwheel_enc = self.lwheel_dir * self.leftCtr * .01 # cm's moved
      rwheel_enc = self.rwheel_dir * self.rightCtr * .01 # cm's moved

      self.lwheel_encs = self.lwheel_encs[1:] + [lwheel_enc]
      self.rwheel_encs = self.rwheel_encs[1:] + [rwheel_enc]

      # History of past three encoder reading
      time_curr_update = rospy.Time.now()
      dt = (time_curr_update - self.time_prev_update).to_sec()

      # Compute angular velocity in rad/s
      lwheel_enc_delta = abs(self.lwheel_encs[-1]) - abs(self.lwheel_encs[-2])
      rwheel_enc_delta = abs(self.rwheel_encs[-1]) - abs(self.rwheel_encs[-2])
      lwheel_angular_vel_enc = self.enc_2_rads(lwheel_enc_delta) / dt
      rwheel_angular_vel_enc = self.enc_2_rads(rwheel_enc_delta) / dt

      # Adjust sign
      if self.lwheel_encs[-1] < 0: lwheel_angular_vel_enc = -lwheel_angular_vel_enc
      if self.rwheel_encs[-1] < 0: rwheel_angular_vel_enc = -rwheel_angular_vel_enc
      self.lwheel_angular_vel_enc_pub.publish(lwheel_angular_vel_enc)
      self.rwheel_angular_vel_enc_pub.publish(rwheel_angular_vel_enc)
            

      self.time_prev_update = time_curr_update

    else: # Running in simulation -- blindly copy from target assuming perfect execution
      self.lwheel_angular_vel_enc_pub.publish(self.lwheel_angular_vel_control)
      self.rwheel_angular_vel_enc_pub.publish(self.rwheel_angular_vel_control)
      
  def spin(self):
    rospy.loginfo("Start fin_state_updater")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)

    while not rospy.is_shutdown():
      self.update()
      rate.sleep()
    rospy.spin()

  def shutdown(self):
    rospy.loginfo("Stop fin_state_updater")
    # Stop message
    self.lwheel_angular_vel_enc_pub.publish(0)
    self.rwheel_angular_vel_enc_pub.publish(0)
    rospy.sleep(1)

def main():
  encoder_publisher = WheelEncoderPublisher();
  encoder_publisher.spin()

if __name__ == '__main__':
  main(); 


