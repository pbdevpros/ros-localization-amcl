#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <thread>
#include <chrono>
#include <mutex>

// Define a global client that can request services
ros::ServiceClient client;
std::mutex mtxMoving ; 
bool isMoving = false; // triggered when white ball is seen

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("\nlin_x " + std::to_string(lin_x)  + "\nang_z " + std::to_string(ang_z) );    

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)) 
        ROS_ERROR("Failed to move robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int lim_left = round(img.step * 0.30);
    int lim_right = round(img.step * 0.70);
    bool isBallInImage = false;

    // Loop through each pixel in the image and check if there's a bright white one
    std::unique_lock<std::mutex> lck(mtxMoving, std::defer_lock);
    if (lck.try_lock()) {
    for (int i = 0 ; i < (img.step * img.height - 2) ; i=i+3 ) {
        if ( 	img.data[i] == white_pixel && 
		img.data[i+1] == white_pixel && 
		img.data[i+2] == white_pixel 	) {
            isMoving = true;
	    isBallInImage = true;
            int pos = i % img.step;
	
            // Then, identify if this pixel falls in the left, mid, or right side of the image
            // Depending on the white ball position, call the drive_bot function and pass velocities to it
            if (pos > lim_left) { 
               if (pos < lim_right) { // middle
                    drive_robot(0.1, 0.0);
               } else { // right
                    drive_robot(0.01, -0.05);
               }
            } else { // left
                drive_robot(0.01, 0.05);
            }
  	    break;
        }
    }
    // Request a stop when there's no white ball seen by the camera
    if (!isBallInImage) {
	drive_robot(0.00, 0.00);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    isMoving = false;
    }

    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    ROS_INFO_STREAM("Setting up client service.");
    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ROS_INFO_STREAM("Subscribing to /camera/rgb/image_raw topic, not rgb_camera/image_raw ");
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
