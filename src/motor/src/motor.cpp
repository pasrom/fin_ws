
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/UInt64MultiArray.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

#include <sstream>
#include <pigpio.h>

#include <stdint.h>
#include <math.h>
#include <time.h>
#include <float.h>

#include "rotary_encoder.hpp"

#define rate 100
#define Motor1Plus 19
#define Motor1Minus 20
#define Motor2Plus 21
#define Motor2Minus 26
#define radius 0.0325
#define incr 20.0

static uint64_t posL = 0;
static uint64_t posR = 0;
static uint64_t startL = 0;
static uint64_t startR = 0;
static int16_t directionL = 0;
static int16_t directionR = 0;
static int cnt = 0;

void callbackLeftWheel(const std_msgs::Int16::ConstPtr& msg)
{
  //ROS_INFO("Left: [%d]", msg->data);
  if (msg->data > 0){
  gpioPWM(Motor1Minus, 0);
  gpioPWM(Motor1Plus, abs(msg->data));
  directionL = 1;
  } else {
  gpioPWM(Motor1Plus, 0);
  gpioPWM(Motor1Minus, abs(msg->data));
  directionL = -1;
  }
  if (msg->data == 0) {
    gpioPWM(Motor1Minus, 0);
    gpioPWM(Motor1Plus, 0);
    directionL = 1;
  }
}

void callbackRightWheel(const std_msgs::Int16::ConstPtr& msg)
{
  //ROS_INFO("Right : [%d]", msg->data);
  if (msg->data > 0){
    gpioPWM(Motor2Minus, 0);
    gpioPWM(Motor2Plus, abs(msg->data));
    directionR = 1;
  } else {
    gpioPWM(Motor2Plus, 0);
    gpioPWM(Motor2Minus, abs((int16_t)((float)msg->data*0.9)));
    directionR = -1;
  }
  if (msg->data == 0) {
    gpioPWM(Motor2Minus, 0);
    gpioPWM(Motor2Plus, 0);
    directionR = 1;
  } 
}


void callbackL(int way)
{
   posL += way;
   //std::cout << "posL=" << posL << std::endl;
}

void callbackR(int way)
{
   posR += way;
  //std::cout << "posR=" << posR << std::endl;
}


int main(int argc, char **argv)
{
  if (gpioInitialise() < 0) return -1;
  ros::init(argc, argv, "robot_motor");
  ROS_INFO("robot_motor init done!");
  ros::NodeHandle n;

  ros::Publisher encoderL_pub = n.advertise<std_msgs::UInt64>("robot/inkL", 50);
  ros::Publisher encoderR_pub = n.advertise<std_msgs::UInt64>("robot/inkR", 50);
  ros::Publisher encoderProTL_pub = n.advertise<std_msgs::UInt64>("robot/inkProTL", 50);
  ros::Publisher encoderProTR_pub = n.advertise<std_msgs::UInt64>("robot/inkProTR", 50);
  ros::Publisher lwheel_angular_vel_enc_pub = n.advertise<std_msgs::Float32>("robot/lwheel_angular_vel_enc", 50);
  ros::Publisher rwheel_angular_vel_enc_pub = n.advertise<std_msgs::Float32>("robot/rwheel_angular_vel_enc", 50);
  ros::Subscriber wheelL_sub = n.subscribe("robot/leftWheel", 50, callbackLeftWheel);
  ros::Subscriber wheelR_sub = n.subscribe("robot/rightWheel", 50, callbackRightWheel);
  ros::Rate loop_rate(rate);

  re_decoder decL(23, callbackL);
  re_decoder decR(22, callbackR);

  std_msgs::UInt64 outPosL;
  std_msgs::UInt64 outPosR;
  std_msgs::UInt64 outPosProTL;
  std_msgs::UInt64 outPosProTR;
  std_msgs::Float32 outLwheel_enc;
  std_msgs::Float32 outRwheel_enc;
  outPosL.data = 0;
  outPosR.data = 0;
  struct timespec timeStruct;
  float timeOldS = 0.0;

  while (ros::ok())
  {
    //arr.data= [posL,posR];
    outPosL.data = posL;
    outPosR.data = posR;
	  if(cnt % (int)(rate*0.2) == 0){
      outPosProTL.data = posL - startL;
      outPosProTR.data = posR - startR;
      clock_gettime(CLOCK_MONOTONIC, &timeStruct);
      float timeNowS = (float)(timeStruct.tv_sec) + (float)(timeStruct.tv_nsec) / 1000000000.0        ;
      float dt = timeNowS-timeOldS;
      //std::cout << "dt=" << dt;
      //std::cout << "posDif=" << posL - startL << std::endl;
      outLwheel_enc.data = (float)directionL * (posL - startL) / incr *2*M_PI/dt;
      outRwheel_enc.data = (float)directionR * (posR - startR) / incr *2*M_PI/dt;
	  //std::cout << "posDif=" << posL - startL << std::endl;
      encoderProTL_pub.publish(outPosProTL);
      encoderProTR_pub.publish(outPosProTR);
      lwheel_angular_vel_enc_pub.publish(outLwheel_enc);
      rwheel_angular_vel_enc_pub.publish(outRwheel_enc);
      startL = posL;
      startR = posR;
      timeOldS = timeNowS;
      cnt = 0;
    }
    clock_gettime(CLOCK_MONOTONIC, &timeStruct);
    encoderL_pub.publish(outPosL);
    encoderR_pub.publish(outPosR);
    

    ros::spinOnce();
    loop_rate.sleep();
	cnt++;
  }
  gpioPWM(Motor1Plus, 0);
  return 0;
}

