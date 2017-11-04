#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <time.h>

class Random
{
	public:
	
		Random();
		void ScanAndMove(const sensor_msgs::LaserScan::ConstPtr& scan);
		float rad2deg(float x);
	private:
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;											
		ros::Subscriber sub;
		ros::ServiceServer service_;
		
		geometry_msgs::Twist base_cmd;	
																//Object to the twist class
	    	
		

};

Random::Random()

{
	base_cmd.linear.x=base_cmd.linear.y=base_cmd.angular.z=0;                                                                                       //Initialize all the movement variables																//Variable to store the last angle when encountered an obstacle
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);                                                           
    sub = nh_.subscribe("/scan", 1000, &Random::ScanAndMove, this);                                         
	
	
	
	
	}
	
void Random::ScanAndMove(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	float sum = 0;
	float mean = 0;
	bool status = 1;                                                                                                                                //Variable to change between linear and angular movements
	int available = 1;	 
	double time = 0.0;
	double secs = ros::Time::now().toSec();															//Count for the degrees
	while (available < 2|| ros::Time::now().toSec() - secs < ros::Duration(30.0).toSec())																//Check if the program is available
	{
                //count = scan->scan_time / scan->time_increment;                                                                                         //Initialize the count variable
		for(int i = 0; i < scan->ranges.size(); i++)														//Start a "for-loop" to check all the data of the lidar
		{		
            float degree = rad2deg(scan->angle_min + scan->angle_increment * i);                                                          //Convert the angle given by the lidar (radian) into degree
            //ROS_INFO ("degree : %f", degree);
            //if ((degree > 122) and (degree < 238))                                                                                          //Check if the angle is in the range [149; 211] which corresponds to one need by the robot to pass between two obstacles at 50cm
			//{
				if(scan->ranges[i] < 0.50000)												//Check if the distance detected is less than 30cm
				{
                    status = 0;												//If it is the case, change the status variable and store a lastang value corresponding to the current angle
					ROS_INFO ("Hola");
					sum = sum + (degree-180)*scan->ranges[i];
				}
			//}
		}
		//ROS_INFO ("Value : %f", sum);
        if (status == 1)															//Check the status of the robot (1=no obstacles, 0 = obstacles met)
		{
            base_cmd.linear.x = 0.25;													//Set the movement variables for moving forward
			base_cmd.angular.z = 0;
            ROS_INFO ("Move");														//Print  a message on the screen to have a visual information
			cmd_vel_pub_.publish(base_cmd);													//Publish the variables to start moving		
		}
		else												
		{
            base_cmd.linear.x = 0;														//Stop the robot to moving forward
            if(sum >= 0)															//Check if the last met angle is on the a left part or right part of the robot
			{
                base_cmd.angular.z = 0.4;												//Set the variables for right rotation if lastang is on the left side
				ROS_INFO ("Left Rotation");												//Print  a message on the screen to have a visual information	
			}
			else
			{
                base_cmd.angular.z = -0.4;												//Set the variables for left rotation if lastang is on the right side
                ROS_INFO ("Right Rotation");												//Print  a message on the screen to have a visual information
			}
			cmd_vel_pub_.publish(base_cmd);													//Publish the twist commands
            ros::Duration(0.05).sleep();													//Add a delay to the rotation to make it possible
		}
        base_cmd.angular.z = 0;															//Stop the robot rotation
		cmd_vel_pub_.publish(base_cmd);														//Publish the stop command
        available++; 
        ROS_INFO("%f",ros::Time::now().toSec() - secs);																//Set available to false to do the loop only once
	}
	//base_cmd.linear.x = 0;
	//base_cmd.angular.z = 0;
	//cmd_vel_pub_.publish(base_cmd);
	sub.shutdown();
	cmd_vel_pub_.shutdown();
	
	
	}

float Random::rad2deg(float x)
{
	return (x)*180./M_PI;
	
	
	}	
	
	
int main(int argc, char **argv)																
{
    ros::init(argc, argv, "random");                                                                                                               														
	
	Random random;

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

    	return 0;
}
