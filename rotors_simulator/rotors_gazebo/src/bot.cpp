#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <iterator>
#include <random>
#include <thread>
#include "tf/transform_datatypes.h"
#include "ros/time.h"
#include <angles/angles.h>
#include <thread>
#include <vector>
#include <stdlib.h>
#include <array>

using namespace std;

#define PI 3.14159
enum State {MOVE,TURN};

class iRobot{
	ros::NodeHandle nh;
	std::default_random_engine generator;
	std::normal_distribution<double> distribution;

public:
	
	double velocity;			// Forward Moving Velocity
	double flipDuration;		// Time to headRotation degree turn
	double noiseDuration;		// Time to random noise turn
	double rotationTime;		// Time taken to turn 
	double maxRotation;			// Max noise angle possible
	int robotID;				// Robot ID Number
	bool turning;				// Flag to tell the current state of bot 
	int time;					// Timer clock of bot
	double topRotation;			// Rotation angle by tap
	double headRotation;		// Rotation angle by head on collision
	double yaw;					// Current Yaw value of the bot
	ros::Publisher cmdVel;		// cmd_vel topic publisher
	ros::Subscriber odom;		// odom subscriber
	ros::Subscriber contactSub;	// cotact subscriber
	State state;				// Current state of bot
	int startTime;				// Start time of bot wrt ROS clock
	bool firstCallBack;			// First odom callback monitor to intialise startTime
	geometry_msgs::Twist Vel;	// cmd_vel msg type
	int prevTime;				// Last time turn was called
	int prevContactTime;		// Last time a contact was made
	int contact;				// Contact type , 1 - head-on 2 - tap 


	// Generates a random number based on gaussian distribution 
	double getRotationAngle(){
		double num = distribution(generator);
		num = (num < 0.3)?0.3:num;
		return (num > 1.0)?maxRotation:(num < -1.0)?-maxRotation:num*maxRotation;
	}

	// Fuction to switch states and publish forward command otherwise
	void start(){
		state = MOVE; // Mode set

		if(firstCallBack){		// If first callback of the yaw state
			firstCallBack = false;
			startTime = ros::Time::now().toSec();		// start the bot and initialise startTime
		}
		
		// while(nh.ok() and !contact){ // Checks if the bot is in contact of any bot or being tapped
		while(nh.ok()){

			ros::Rate loop_rate(10);

			// Check for headRotation time
			if(time != prevTime and time % (int)flipDuration == 0 and not turning){
				state = TURN;
				prevTime = time;
				// ROS_INFO("Flip T: %d %d",time,startTime);
				turning = true;
				turnBy(headRotation);
				return;
			}
			else
			//Check for Noise duration
				if(time != prevTime and (time)%(int)noiseDuration == 0 and not turning){
					state = TURN;
					prevTime = time;
					// ROS_INFO("Noise");
					turning = true;
					turnBy(getRotationAngle(),true);
					return;
				}
			// State shift 
			state = (turning)?TURN:MOVE;
			//Straight moving
			if(state == MOVE){
				Vel.linear.x = velocity;
				Vel.linear.y = 0;
				Vel.linear.z = 0;
				Vel.angular.x = 0;
				Vel.angular.y = 0;
				Vel.angular.z = 0;
				cmdVel.publish(Vel);
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	// Turn the bot
	void turnBy(double turn_angle, bool noise = false)
	{
	    turning = true;
	    ros::Rate rate(50.0);
	    // Vel.angular.z = 0.4;	// Around 1-2 degree of rotation error.
	    double stTime = ros::Time::now().toSec();
	    ////////////	PID Parameters	/////////////
	    double Kp = 1.45,Ki = 0.01,Kd = 0.01;
	    double lastTime = time;
	    array<double,10> I_values = {{0}};	// Stores last 10 values for integral
	    double last_angle = yaw;
	    double angle = 0;
	    int i = 0;					// Circular input in I_values
	    double dt = 0.0;
	    double error;

	    Vel.linear.x = (noise)?velocity:0.;
		Vel.linear.y = 0;
		Vel.linear.z = 0;
		Vel.angular.x = 0;
		Vel.angular.y = 0;
		turn_angle *= PI/180.;

	    do{
	    	if(!noise){
		    	error = turn_angle - angle;
		    	I_values[i++%10] = error;
		    	double sum = 0.;
		    	std::for_each(I_values.begin(), I_values.end(),[&sum](double v) { sum += v; });
		    	dt = lastTime - ros::Time::now().toSec();
		    	lastTime = ros::Time::now().toSec();
		    	Vel.angular.z = Kp*error + Ki*sum + Kd*error/dt;
		    	// ROS_INFO("Error : %f",error);
	    	}
	    	else{
	    		Vel.angular.z = 0.5;		// Turning happens instanteneously thus PID not required
	    	}
	        cmdVel.publish(Vel);
	        ros::spinOnce();
	        rate.sleep();

	        // Compute the amount of rotation since the last loop
	        angle += angles::normalize_angle(yaw - last_angle);
	        last_angle = yaw;
	    }while (((noise and angle < turn_angle) or (!noise and  abs(Vel.angular.z) > 0.01)) and nh.ok());
	    Vel.angular.z = 0.;
	    turning = false;
	    // if(!noise)
	    	// ROS_INFO("Total Time : %f %f",ros::Time::now().toSec() - stTime, stTime);
	    start(); 				// Hand back control to Move 
	}


	// updates bot timer
	void timer(){
		while(nh.ok()){
			ros::Rate loop_rate(10);

			if(firstCallBack) // If the startTime is not initialised skip
				continue;

			if(!turning)	  // Update timer of the bot
				time = ros::Time::now().toSec() - startTime;
			else			// If turning , stop clock update and update startTime to maintain the timer at steady state once turning is done
				startTime = ros::Time::now().toSec() - time ;

			loop_rate.sleep();
		}
	}


	// update current yaw value of the bot
	void OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg){
		
		// if(firstCallBack){		// If first callback of the yaw state
		// 	firstCallBack = false;
		// 	startTime = ros::Time::now().toSec();		// start the bot and initialise startTime
		// }	

		tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
	    tf::Matrix3x3 m(q);
	    double roll, pitch;

	    m.getRPY(roll, pitch, yaw);					// Get yaw value
	}

	// Contact string callback
	void ContactCallBack(const std_msgs::String::ConstPtr& msg){
		
		// Read string with contact condition of all bots and pick the robotID status
		contact = (msg->data)[robotID] - '0';

		if(contact and prevContactTime != time){
			prevContactTime = time;
			if(contact == 1)
				turnBy(headRotation);		// head collision
			else if(contact == 2)
				turnBy(topRotation);		// Tapping
		}
	}

	iRobot(string ID,ros::NodeHandle n){
		nh = n;
		distribution = std::normal_distribution<double>(0.0,1.0);	// Normal distribution with 0 mean 1 variance
		nh.param("x_velocity",velocity,0.33);						// Initialise forward velocity with 0.33 default if parameter is not passed
		nh.param("straight_duration",flipDuration,20.);				// Initialise 180 turn duration with 20 default if parameter is not passed
		nh.param("noise_duration",noiseDuration,5.);				// Initialise noise duration with 5 default if parameter is not passed
		nh.param("rotation_time",rotationTime,2.15);				// Initialise turn duration with 2.15s default if parameter is not passed
		nh.param("rotation_time",maxRotation,10.);					// Initialise maximum noise angle with 20 default if parameter is not passed
		state = MOVE;
		robotID = ID.back() - '0';									// string to integer
		topRotation = 45.;
		headRotation = 180;
		yaw = 0.;
		turning = false;
		time = 0;
		startTime = 0;
		contact = 0;
		prevContactTime = -1;										
		firstCallBack = true;
		prevTime = 0;

		// Publisher and Subscriber Initialisations
		cmdVel = nh.advertise<geometry_msgs::Twist>("/"+ID+"/cmd_vel", 1);
		odom = nh.subscribe("/"+ID+"/odom", 30, &iRobot::OdomCallBack,this);
		contactSub = nh.subscribe("/contact", 30, &iRobot::ContactCallBack,this);

		// Start Timer
		std::thread timerThread(&iRobot::timer,this);
		timerThread.detach();		// Deattach thread
	}
};

bool isActive = false;

iRobot *bot; 
bool running = false;

void ActivateCallBack(const std_msgs::Bool::ConstPtr& activate){
	isActive = activate->data;
	if(isActive and not running){
		bot->start();
		running = true;
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "groundbot_node");
	ros::NodeHandle n("~");
	string ID,def = "robot1";
	//cout<<"ID:"<<ID<<" "<<def;
	ros::Subscriber active;
	active = n.subscribe("/activate", 10, ActivateCallBack);
	n.param("robotID",ID,def);		// Read robot ID (robotX : x being the robot number) defaults to 5
	bot = new iRobot(ID,n);
	//bool running = false;
	running = true;
	bot->start();
	// while(n.ok() && running==false) {
	// 	ros::spinOnce();
	// }
	// while(n.ok()) {
	// 	ros::spinOnce();
	// }
	while(n.ok()){}
	return 0;
}