#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "geometry_msgs/Twist.h"

class Drive
{
public:
  Drive();
  bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res);
private:
  ros::NodeHandle n;
  ros::ServiceServer service;
  ros::Publisher motor_control_publisher;
  
};

Drive::Drive()
{
  motor_control_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  service = n.advertiseService("/ball_chaser/command_robot",&Drive::handle_drive_request,this);

}

bool Drive::handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
  geometry_msgs::Twist motor_command;
  motor_command.linear.x = req.linear_x;
  motor_command.angular.z = req.angular_z;
  motor_control_publisher.publish(motor_command);
	
  //returning a response message
  res.msg_feedback = "SET linear x " + std::to_string(motor_command.linear.x) + " angular z " + std::to_string(motor_command.angular.z)+ "\n";
  return true;
		
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"drive_bot");
  Drive Obj;
  
  ROS_INFO("Ready to move the robot");

  ros::spin();
	
  return 0;
	
}
