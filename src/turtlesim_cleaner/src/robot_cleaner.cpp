#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

const double pi= 3.14159265359;
const double x_max = 11.0;
const double y_max = 11.0;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

//method to move robot straight
void move_straight(double speed,double distance,bool isForward);
void rotate(double angular_speed,double angle,bool clockwise);//clockwise negative angle, counter clockwise positive angle
//ros understand radiant we have to convert degree to radiant
double degree2radiant(double angele2radiant);  
void desired_angle(double desire_angle);
void pose_callback(const turtlesim::Pose::ConstPtr& pose_message);
double getdistance(double x1,double y1,double x2,double y2);
void move2goal(turtlesim::Pose goal_pose, double distance_tolerence);
void grid_clean();



int main(int argc,char **argv)
{
	//Initiate new ROS node named "talker"
	ros::init(argc,argv,"robot_cleaner");
	ros::NodeHandle n;

	double speed;
	double distance;
	bool isForwardd;

	double angle_speed;
	double angle;
	bool clockwise;


     //publishing the velocity message
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, pose_callback);

	/*std::cout<<"enter speed: ";
	std::cin>>speed;
	std::cout<<"enter distance: ";
	std::ci n>>distance;
	std::cout<<"forward: ";
	std::cin>>isForwardd;
	move_straight(speed,distance,isForwardd);

	std::cout<<"angular speed: ";
	std::cin>>angle_speed;
	std::cout<<"angle: ";
	std::cin>>angle;
	std::cout<<"clockwise: ";
	std::cin>>clockwise;
	rotate(degree2radiant(angle_speed),degree2radiant(angle),clockwise);
    
    desired_angle(degree2radiant(120));
    ros::Rate loop_rate(0.5);
    loop_rate.sleep();
    desired_angle(degree2radiant(-90)); 
    ros::spin();*/

    /*
    ros::Rate loop_rate(0.5);
    turtlesim::Pose goal_pose;
    goal_pose.x=1;
    goal_pose.y=1;
    goal_pose.theta=3;
    move2goal(goal_pose, 0.01);
    */
    ros::Rate loop_rate(0.5);
   
    grid_clean();

    loop_rate.sleep();
    ros::spin();


	return 0;
}



void move_straight(double speed,double distance,bool isForward)
{
	//distance=speed*time
	//defining the twist message 
	geometry_msgs::Twist vel_msg;
	//set random linear velocity in x-axis
	if(isForward)
	{
		vel_msg.linear.x=abs(speed);
	}
	else{
		vel_msg.linear.x=-abs(speed);
	}
	//set y,z axis in 0 because move in one dimension
	vel_msg.linear.y=0; 
	vel_msg.linear.z=0;

	//set random angular velocity in the y-axis 
	vel_msg.angular.x=0; 
	vel_msg.angular.y=0;
	vel_msg.angular.z=0;

	//t0=current time
	//intila time
	 double t0= ros::Time::now().toSec();
	 double current_distance = 0;
	 //message send per 10ms
	 ros::Rate loop_rate(10);
	 //loop
	//publish the velocity
	//estimate the current_distance=speed*(t1-t0)
	//current distance moved by the robot <=distance because move the robot until that distance
	 do
	 {
	 	velocity_publisher.publish(vel_msg); //in x coordiante it will piblish the msg speed
	 	double t3=ros::Time::now().toSec();
	 	//this current distance is for terminate the loop but the speed is assigned with linear x speed will uplish untill terminate
	 	current_distance=speed*(t3-t0);
	 	//to succesfully publish message
	 	ros::spinOnce();
	 	loop_rate.sleep();

	 }while(current_distance<distance);
	 //force the robot to stop if it reaches the distance
	 vel_msg.linear.x=0;
	 velocity_publisher.publish(vel_msg);
	



}

void rotate(double angular_speed,double angle,bool clockwise)
{

    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x=0;
	vel_msg.linear.y=0; 
	vel_msg.linear.z=0;

	//set random angular velocity in the y-axis 
	 
	vel_msg.angular.x=0;
	vel_msg.angular.y=0;
	


    
    if(clockwise)
	{
		vel_msg.angular.z=-abs(angular_speed);
	}
	else{
		vel_msg.angular.z=abs(angular_speed);
	}
	//set y,z axis in 0 because move in one dimension
	

	//t0=current time
	//intila time
	 double t0= ros::Time::now().toSec();
	 double current_angle = 0;
	 //message send per 10ms
	 ros::Rate loop_rate(10);
	 //loop
	//publish the velocity
	//estimate the current_distance=speed*(t1-t0)
	//current distance moved by the robot <=distance because move the robot until that distance
	 do
	 {
	 	velocity_publisher.publish(vel_msg); //in x coordiante it will piblish the msg speed
	 	double t3=ros::Time::now().toSec();
	 	//this current distance is for terminate the loop but the speed is assigned with linear x speed will uplish untill terminate
	 	current_angle=angular_speed*(t3-t0);
	 	//to succesfully publish message
	 	ros::spinOnce();
	 	loop_rate.sleep();

	 }while(current_angle<angle);
	 //force the robot to stop if it reaches the distance
	 vel_msg.angular.z =0;
	 velocity_publisher.publish(vel_msg);

}


double degree2radiant(double angele2radiant)
{

   return angele2radiant *pi/180.0;

}

void desired_angle(double desire_angle)
{
	double relative_angle_radians = desire_angle - turtlesim_pose.theta;
    bool clockwise= ((relative_angle_radians<0)?true:false);

    rotate(abs(relative_angle_radians), abs(relative_angle_radians),clockwise);

}

void pose_callback(const turtlesim::Pose::ConstPtr & pose_message)
{

	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}
double getdistance(double x1,double y1,double x2,double y2)
{
	return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

void move2goal(turtlesim::Pose goal_pose, double distance_tolerence)
{

    geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double E = 0.0;
	do{
		/****** Proportional Controller ******/
		//linear velocity in the x-axis
		double Kp=1.0;
		double Ki=0.02;
		//double v0 = 2.0;
		//double alpha = 0.5;
		double e = getdistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		double E = E+e;
		//Kp = v0 * (exp(-alpha)*error*error)/(error*error);
		vel_msg.linear.x = (Kp*e);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =4*(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getdistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerence);
	cout<<"end move goal"<<endl;
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}  

void grid_clean()
{
	ros::Rate loop(0.5);
    turtlesim::Pose pose;
    pose.x=1;
    pose.y=1;
    pose.theta=0;
    move2goal(pose,1.0);
    loop.sleep();
    desired_angle(0);
    loop.sleep();


    move_straight(2.0,4.0,true);
    loop.sleep();
    rotate(degree2radiant(3),degree2radiant(90),false);
    loop.sleep();
    move_straight(2.0,3.0,true);
    loop.sleep();
    rotate(degree2radiant(3),degree2radiant(150),false);
    loop.sleep();
    move_straight(2.0,5.0,true);

    //double distance = getdistance(turtlesim_pose.x, turtlesim_pose.y, x_max, y_max);



}