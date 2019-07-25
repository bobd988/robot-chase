#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service and pass the requested motor commands
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    bool isFound = false;
    int row = 0;
    int step = 0;
    int i = 0;
   
    for (row = 0; row < img.height && isFound == false; row++)
    {
        for (step = 0; step < img.step && isFound == false; ++step)
        {   
            i = (row*img.step)+step;
            //ROS_INFO("row: %d, step: %d, i: %d", row, step, i);
            //if (img.data[i] == white_pixel)
           int red_channel = img.data[i];
           int green_channel = img.data[i+1];
           int blue_channel = img.data[i+2];
           //if (img.data[i] == white_pixel)
           if (red_channel == 255 && green_channel == 255 && blue_channel == 255)
            {   
                isFound = true;
                ROS_INFO("found white i:%d",i);
                
            }
		}
    }
    if (isFound)
    {
        // split left, mid, or right side 
        int imgThird = img.width/3;
        int col = step/3;
        if (col < imgThird) 
        {
            drive_robot(0.1, 0.1);
            ROS_INFO("Move left");
        } 
        else if (col >= imgThird && col < 2*imgThird)
        {
            drive_robot(0.5, 0.0);
            ROS_INFO("Move straight");
        }
        else if (col >= 2*imgThird)
        {
            drive_robot(0.1, -0.1);
            ROS_INFO("Move right");
        }
        
    }
  	else 
    {
        drive_robot(0.0, 0.0);
        ROS_INFO("Not found - STOP");
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}