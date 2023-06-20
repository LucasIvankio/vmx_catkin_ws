#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

ros::Publisher joint_state_pub;


void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    // Convert the velocity command to individual wheel commands
    double linear_vel = cmd_vel->linear.x;
    double angular_vel = cmd_vel->angular.z;

    // Calculate the wheel velocities
    double wheel_front_left_vel = linear_vel - angular_vel;
    double wheel_front_right_vel = linear_vel + angular_vel;
    double wheel_rear_left_vel = linear_vel - angular_vel;
    double wheel_rear_right_vel = linear_vel + angular_vel;


    // Create a JointState message
    sensor_msgs::JointState joint_state;
    joint_state.name.push_back("front_left_wheel_hinge");
    joint_state.name.push_back("front_right_wheel_hinge");
    joint_state.name.push_back("back_left_wheel_hinge");
    joint_state.name.push_back("back_right_wheel_hinge");
    joint_state.velocity.resize(4);
    joint_state.position.resize(4);



    // Assign the wheel velocities to the JointState message
    joint_state.velocity[0] = wheel_front_left_vel;
    joint_state.velocity[1] = wheel_front_right_vel;
    joint_state.velocity[2] = wheel_rear_left_vel;
    joint_state.velocity[3] = wheel_rear_right_vel;

    // Publish the JointState message
    joint_state_pub.publish(joint_state);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "skid_steer_mapping_node");
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmd_vel_callback);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::spin();

    return 0;
}