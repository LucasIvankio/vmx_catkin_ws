#include "TitanDriver_ros_wrapper.h"
#include "navX_ros_wrapper.h"
#include "Cobra_ros.h"
#include "Sharp_ros.h"
#include "Servo_ros.h"
#include "Ultrasonic_ros.h"
#include "IOwd_ros.h"
#include "DI_ros.h"
#include "DO_ros.h"
#include <unistd.h>
#include <ros/ros.h>


double ultrasonic_cm;
double sharp_cm;

double servo_angle;

void servo_angle_callback(const std_msgs::Float32::ConstPtr& msg)
{
   servo_angle = msg->data;
}

void sharp_cm_callback(const std_msgs::Float32::ConstPtr& msg)
{
   sharp_cm = msg->data;
   ROS_INFO_STREAM("Distance SHARP = " << sharp_cm << "cm" "\n");
}

// Returns the distance value reported by the Ultrasonic Distance sensor
void ultrasonic_cm_callback(const std_msgs::Float32::ConstPtr& msg)
{
   ultrasonic_cm = msg->data;
}

int main(int argc, char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
   ros::init(argc, argv, "main_node");
  
   ros::AsyncSpinner spinner(4); //Allows callbacks from multiple threads; spins asynchronously using 4 threads
   spinner.start(); //Starts this spinner spinning asynchronously
   
   ros::NodeHandle nh; //Internal reference to the ROS node that the program will use to interact with the ROS system
   VMXPi vmx(true, (uint8_t)50); //Realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz

   ros::ServiceClient setAngle;
   ros::Subscriber servo_angle_sub;

   ServoROS servo(&nh, &vmx, 12);

   setAngle = nh.serviceClient<vmxpi_ros::Float>("channel/12/servo/set_angle");

   vmxpi_ros::Float msg;

   // ros::Subscriber sharp_sub;
   ros::Subscriber sharp_sub;

   // UltrasonicROS ultrasonic(&nh, &vmx, 8, 9); //channel_index_out(8), channel_index_in(9)
   // ultrasonic.Ultrasonic(); //Sends an ultrasonic pulse for the ultrasonic object to read

   SharpROS sharp(&nh, &vmx);

   // Use these to directly access data
   // uint32_t raw_distance = ultrasonic.GetRawValue(); // returns distance in microseconds
   // or can use
   // uint32_t cm_distance = ultrasonic.GetDistanceCM(raw_distance); //converts microsecond distance from GetRawValue() to CM


   ros::ServiceClient set_m_speed;

   TitanDriverROSWrapper titan(&nh, &vmx);

   vmxpi_ros::MotorSpeed msg_mt;
   msg_mt.request.motor=0;

   set_m_speed = nh.serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");
   
   servo_angle_sub = nh.subscribe("channel/12/servo/angle", 1, servo_angle_callback);

   // Subscribing to Ultrasonic distance topic to access the distance data
   // ultrasonicCM_sub = nh.subscribe("channel/9/ultrasonic/dist/cm", 1, ultrasonic_cm_callback); //This is subscribing to channel 9, which is the input channel set in the constructor

   sharp_sub = nh.subscribe("channel/22/sharp_ir/dist", 1, sharp_cm_callback);
   while(ros::ok())
   {
      uint32_t dist_cm = sharp.GetIRDistance();
      float angle=0;
      if(dist_cm <150.0 && dist_cm > -150.0)angle=dist_cm;
      msg.request.data = angle;
      setAngle.call(msg);
      msg_mt.request.speed=angle/100;
      set_m_speed.call(msg_mt);
   }
   // ros::spin(); //ros::spin() will enter a loop, pumping callbacks to obtain the latest sensor data
   
   ROS_INFO("ROS SHUTDOWN");
   ros::waitForShutdown();
   return 0;
}
