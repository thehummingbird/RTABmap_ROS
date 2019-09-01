#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>

class ProcessImage
{
public:
  ProcessImage();
  void process_image_callback(const sensor_msgs::Image img);

private:
  ros::NodeHandle n;
  ros::Subscriber sub_camera;
  ros::ServiceClient drive_client;
    
};

ProcessImage::ProcessImage()
{
  sub_camera = n.subscribe("camera/rgb/image_raw",10,&ProcessImage::process_image_callback,this);
  drive_client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

}

void ProcessImage::process_image_callback(const sensor_msgs::Image img)
{
  float pixels_left = 0;
  float pixels_mid = 0;
  float pixels_right = 0;
	
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = 0;
  srv.request.angular_z = 0;
	
  for(int i=0;i<img.height;i++)
  {
    for(int j=0;j<(img.step-2);j+=3)
    {
      if((img.data[(i*(img.step))+j]==255) && (img.data[(i*(img.step))+j+1]==255) && (img.data[(i*(img.step))+j+2]==255))
      {
        if((j<=(img.step/3))&&(j>=0))
        {
          pixels_left++;
        }
        else if ((j<=((img.step/3)*2)) && (j>(img.step/3)))				
        {
          pixels_mid++;
        }
        else if((j<=(img.step)) && (j>((img.step/3)*2)))
        {
          pixels_right++;
        }
        else
        {
          ROS_INFO("Incorrect State\n");
        }
      }
    }
  }
	
  if(pixels_left>pixels_mid)
  {
    srv.request.angular_z = 0.6;
    srv.request.linear_x = 0;
  }
  else if(pixels_right>pixels_mid)
  {
   srv.request.angular_z = -0.6;
  srv.request.linear_x = 0.0;
  }
  else if(pixels_mid >20)
  {
    srv.request.linear_x = 0.6;
    srv.request.angular_z = 0;
  }
  else
  {
    srv.request.linear_x = 0;
    srv.request.angular_z = 0.6;
  }
  if (!drive_client.call(srv))
    ROS_ERROR("Failed to call service safe_move");

}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "process_image");

  ProcessImage Obj;

  ros::spin();

  return 0;

}
