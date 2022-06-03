#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

const double pi = 3.14159265359;


ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

void move_straight(double speed,double distance,bool isForward);
void rotate(double angular_speed,double angle,bool clockwise);
void make_arc(double anglespeed, double radius, double arc_length,bool clockwise);
double degree2radiant(double angle);
void makeb();
void pose_callback(const turtlesim::Pose::ConstPtr& pose_message);
void move2goal(turtlesim::Pose goal_pose, double distance_tolerence);
double getdistance(double x1,double y1,double x2,double y2);



int main(int argc,char **argv)
{

      //Initiate new ROS node named "talker"
	  ros::init(argc,argv,"b_cleaner");
	  ros::NodeHandle n;
	  velocity_publisher=n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);
	  pose_subscriber=n.subscribe("/turtle1/pose", 10, pose_callback);


	  ros:: Rate loop_rate(0.5);

        makeb();
	  loop_rate.sleep();
	  ros::spin();
	  return 0;





}



void move_straight(double speed, double distance,bool isForward)
{

     geometry_msgs::Twist vel_msg;

     if(isForward)
     {

        vel_msg.linear.x=abs(speed);

     }
     else
     {
        vel_msg.linear.x=-abs(speed);

     }

     vel_msg.linear.y=0;
     vel_msg.linear.z=0;

     vel_msg.angular.x=0;
     vel_msg.angular.y=0;
     vel_msg.angular.z=0;

     double t0=ros::Time::now().toSec();
     double cu_dis=0;
     ros::Rate loop_rate(10);
     do
     {
       velocity_publisher.publish(vel_msg);
       double t3=ros::Time::now().toSec();
       cu_dis= speed*(t3-t0);

       ros::spinOnce();
       loop_rate.sleep(); 



     }while(cu_dis < distance);
     vel_msg.linear.x= 0;
     velocity_publisher.publish(vel_msg);
     cout<<turtlesim_pose.x;
     cout<<turtlesim_pose.y;
     cout<<turtlesim_pose.theta;
	
}

double degree2radiant(double angle)
{

      double radiant= angle*pi/180.0;

      return radiant;
	
}

void rotate(double angle_speed, double anglee,bool clockwise)
{

    geometry_msgs::Twist vel_msg;

    if(clockwise)
    {
    	vel_msg.angular.z=-abs(angle_speed);
    }
    else
    {
    	vel_msg.angular.z=abs(angle_speed);
    }

    vel_msg.linear.x=0;
    vel_msg.linear.y=0;
    vel_msg.linear.z=0;

    vel_msg.angular.x=0;
    vel_msg.angular.y=0;


     double t0=ros::Time::now().toSec();
     double cu_ange=0;
     ros::Rate loop_rate(10);
     do
     {
       velocity_publisher.publish(vel_msg);
       double t3=ros::Time::now().toSec();
       cu_ange= angle_speed*(t3-t0);

       ros::spinOnce();
       loop_rate.sleep(); 



     }while(cu_ange < anglee);
     vel_msg.linear.z= 0;
     velocity_publisher.publish(vel_msg);


}

void make_arc(double anglespeed, double radius, double arc_length ,bool clockwise)
{

   geometry_msgs::Twist vel_msg;

   if(clockwise)
   {
    	vel_msg.angular.y=-abs(anglespeed);
   }
   else
   {
    	vel_msg.angular.y=abs(anglespeed);
   }

    vel_msg.linear.x=0;
    vel_msg.linear.y=0;
    vel_msg.linear.z=0;

    vel_msg.angular.x=0;
    vel_msg.angular.z=0;


     double t0=ros::Time::now().toSec();
     double cu_radi=0;
     ros::Rate loop_rate(10);
     do
     {
       velocity_publisher.publish(vel_msg);
       double t3=ros::Time::now().toSec();
       cu_radi= radius*(anglespeed*(t3-t0));

       ros::spinOnce();
       loop_rate.sleep(); 



     }while(cu_radi < arc_length);
     vel_msg.linear.y= 0;
     velocity_publisher.publish(vel_msg);

}

void pose_callback(const turtlesim::Pose::ConstPtr & pose_message)
{

	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}


double getdistance(double x1,double y1,double x2,double y2)
{
	return sqrt(pow(y2-y1,2)+pow(x2-x1,2));
}

void move2goal(turtlesim::Pose goal_pose, double distance_tolerence)
{
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double E=0.0;
	do
	{
		double Kp=1.0;
		double Ki=0.02;

		//get destance
		double e= getdistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y);
		double E = E+e;

		vel_msg.linear.x=Kp*e;
		vel_msg.linear.y=0;
		vel_msg.linear.z=0;

		vel_msg.angular.x=0;
		vel_msg.angular.y=0;
		vel_msg.angular.z=4.0*(atan2(goal_pose.y-turtlesim_pose.y,goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getdistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y)>distance_tolerence);
	cout<<"end move goal"<<endl;
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

void makeb()
{
	ros::Rate loop(0.5);
	rotate(degree2radiant(6),degree2radiant(90),false);
	loop.sleep();
	move_straight(2.0,3.0,true);
	loop.sleep();
	
	loop.sleep();
	move_straight(2.0,6.0,false);
	loop.sleep();
	rotate(degree2radiant(6.0),degree2radiant(90.0),true);
	loop.sleep();
	turtlesim::Pose pose;
	pose.x=5.5;
	pose.y=5.5;
	pose.theta=0.0;
	move2goal(pose,0.01);
	loop.sleep();
	rotate(degree2radiant(6),degree2radiant(150),true);
	turtlesim::Pose goal;
	goal.x=11.0;
	goal.y=11.0;
	goal.theta=0.0;
	move2goal(goal,0.01);


}





