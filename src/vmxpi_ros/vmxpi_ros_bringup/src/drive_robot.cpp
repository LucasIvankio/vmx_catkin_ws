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
#include <sensor_msgs/JointState.h>


ros::ServiceClient set_m_speed;

static double right_encoder=0;


// Callbacks for Encoder Distance values
void motor0Callback(const std_msgs::Float32::ConstPtr& msg)
{
    // right_encoder = msg->data;
    ROS_INFO_STREAM("Encoder distance: " << msg->data << "\n");
}

void joint_state_callback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
    sensor_msgs::JointState my_joint;

    my_joint.velocity = joint_state->velocity;
    my_joint.velocity.resize(4);

    // Extract the joint values from the received JointState message
    for (size_t i = 0; i < joint_state->name.size(); ++i)
    {
        std::string joint_name = joint_state->name[i];
        double joint_vel = my_joint.velocity[i];

        ROS_INFO("Joint: %s, Vel: %f", joint_name.c_str(), joint_vel);

        vmxpi_ros::MotorSpeed msg_mt;
        msg_mt.request.motor=i;
        msg_mt.request.speed=joint_vel;
        set_m_speed.call(msg_mt);
    }

}


int main(int argc, char** argv)
{
    system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
    ros::init(argc, argv, "drive_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4); //Allows callbacks from multiple threads; spins asynchronously using 4 threads
    spinner.start(); //Starts this spinner spinning asynchronously
    
    VMXPi vmx(true, (uint8_t)50);
    

    TitanDriverROSWrapper titan(&nh, &vmx);


    set_m_speed = nh.serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");

    ros::Subscriber motor0_dist = nh.subscribe("titan/encoder0/distance", 1, motor0Callback);


    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 10, joint_state_callback);

   // ros::spin(); //ros::spin() will enter a loop, pumping callbacks to obtain the latest sensor data
   
   ROS_INFO("ROS SHUTDOWN");
   ros::waitForShutdown();
   return 0;
}