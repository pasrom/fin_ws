
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/UInt64MultiArray.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/Int16.h"

#include <sstream>
#include <pigpio.h>

#include <stdint.h>

#include "rotary_encoder.hpp"

#define Motor1Plus 19
#define Motor1Minus 20
#define Motor2Plus 21
#define Motor2Minus 26
static uint64_t posL = 0;
static uint64_t posR = 0;

void callbackLeftWheel(const std_msgs::Int16::ConstPtr& msg)
{
  ROS_INFO("Left: [%d]", msg->data);
  if (msg->data > 0){
  gpioPWM(Motor1Minus, 0);
  gpioPWM(Motor1Plus, abs(msg->data));
  } else {
  gpioPWM(Motor1Plus, 0);
  gpioPWM(Motor1Minus, abs(msg->data));
  }
  if (msg->data == 0) {
    gpioPWM(Motor1Minus, 0);
    gpioPWM(Motor1Plus, 0);
  }
}

void callbackRightWheel(const std_msgs::Int16::ConstPtr& msg)
{
  ROS_INFO("Right : [%d]", msg->data);
  if (msg->data > 0){
    gpioPWM(Motor2Minus, 0);
    gpioPWM(Motor2Plus, abs(msg->data));
  } else {
    gpioPWM(Motor2Plus, 0);
    gpioPWM(Motor2Minus, abs(msg->data));
  }
  if (msg->data == 0) {
    gpioPWM(Motor2Minus, 0);
    gpioPWM(Motor2Plus, 0);
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
  ros::NodeHandle n;

  ros::Publisher encoderL_pub = n.advertise<std_msgs::UInt64>("robot/inkL", 50);
  ros::Publisher encoderR_pub = n.advertise<std_msgs::UInt64>("robot/inkR", 50);
  ros::Subscriber wheelL_sub = n.subscribe("robot/leftWheel", 50, callbackLeftWheel);
  ros::Subscriber wheelR_sub = n.subscribe("robot/rightWheel", 50, callbackRightWheel);
  ros::Rate loop_rate(10);

   re_decoder decL(23, callbackL);
   re_decoder decR(22, callbackR);

  std_msgs::UInt64 outPosL;
  std_msgs::UInt64 outPosR;
  outPosL.data = 0;
  outPosR.data = 0;
  
  while (ros::ok())
  {
    //arr.data= [posL,posR];
    outPosL.data = posL;
    outPosR.data = posR;
    encoderL_pub.publish(outPosL);
    encoderR_pub.publish(outPosR);

    ros::spinOnce();
    loop_rate.sleep();
  }
  gpioPWM(Motor1Plus, 0);
  return 0;
}

