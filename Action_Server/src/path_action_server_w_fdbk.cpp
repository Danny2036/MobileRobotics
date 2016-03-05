#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <Action_Server/path_msgAction.h>

geometry_msgs::Twist g_twist;
ros::Publisher g_twist_commander;
geometry_msgs::Pose g_pose;

const double g_move_speed = 1.0; 
const double g_spin_speed = 1.0;
const double g_sample_dt = 0.01;

class PathActionServer {
private:

	ros::NodeHandle nh_;  

	actionlib::SimpleActionServer<Action_Server::path_msgAction> as_;
    ros::Publisher vel_pub; //Publisher that published to cmd_
    geometry_msgs::Twist g_twist_cmd;
    geometry_msgs::Pose g_current_pose;

    // here are some message types to communicate with our client(s)
    Action_Server::path_msgGoal goal_; // goal message, received from client
    Action_Server::path_msgResult result_; // put results here, to be sent back to the client when done w/ goal
    Action_Server::path_msgFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client


    double sgn(double x);
    double min_spin(double spin_angle);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

    void do_inits();
    void do_halt();
    void do_spin(double spin_ang);
    void do_move(double distance);

public:
    PathActionServer(); //define the body of the constructor outside of class definition

    ~PathActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<Action_Server::path_msgAction>::GoalConstPtr& goal);
};


PathActionServer::PathActionServer() :  as_(nh_, "path_action", boost::bind(&PathActionServer::executeCB, this, _1),false){
	ROS_INFO("in constructor of PathActionServer...");
    // do any other desired initializations here...specific to your implementation
	vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
	do_inits();
    as_.start(); //start the server running
}

void PathActionServer::executeCB(const actionlib::SimpleActionServer<Action_Server::path_msgAction>::GoalConstPtr& goal) {
	ROS_INFO("in executeCB");

	std::vector<double> spin_angle = goal->angle;
	std::vector<double> travel_distance = goal->distance;
	int num_angles = spin_angle.size();
	ros::Rate timer(100.0);
	int i = 0;
	while(ros::ok() && i < num_angles){
		if(as_.isPreemptRequested()){
			ROS_WARN("The goal was cancelled. Dunzo");
			result_.result = false;
			as_.setAborted(result_);
			return;
		}
		feedback_.fdbk = i;
		do_spin(spin_angle[i]);
		do_move(travel_distance[i]);
		do_halt();
		i++;
		timer.sleep();
	}
	do_halt();
	result_.result = true;
	as_.setSucceeded(result_);
}

//signum function: strip off and return the sign of the argument
double PathActionServer::sgn(double x) {
	if (x>0.0) {
		return 1.0; 
	}
	else if (x<0.0) {
		return -1.0;
	} else {
		return 0.0;
	}
}

//a function to consider periodicity and find min delta angle
double PathActionServer::min_spin(double spin_angle) {
	if (spin_angle > M_PI) {
		spin_angle -= 2.0*M_PI;
	}
	if (spin_angle < -M_PI) {
		spin_angle += 2.0*M_PI;
	}
	return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double PathActionServer::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
	double quat_z = quaternion.z;
	double quat_w = quaternion.w;
	double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
	return phi;
}

//and the other direction:
geometry_msgs::Quaternion PathActionServer::convertPlanarPhi2Quaternion(double phi) {
	geometry_msgs::Quaternion quaternion;
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin(phi / 2.0);
	quaternion.w = cos(phi / 2.0);
	return quaternion;
}

void PathActionServer::do_inits() {
	//initialize components of the twist command global variable
	g_twist_cmd.linear.x=0.0;
	g_twist_cmd.linear.y=0.0;    
	g_twist_cmd.linear.z=0.0;
	g_twist_cmd.angular.x=0.0;
	g_twist_cmd.angular.y=0.0;
	g_twist_cmd.angular.z=0.0;  

    //define initial position to be 0
	g_current_pose.position.x = 0.0;
	g_current_pose.position.y = 0.0;
	g_current_pose.position.z = 0.0;

    // define initial heading to be "0"
	g_current_pose.orientation.x = 0.0;
	g_current_pose.orientation.y = 0.0;
	g_current_pose.orientation.z = 0.0;
	g_current_pose.orientation.w = 1.0;
}

void PathActionServer::do_halt() {
	ros::Rate loop_timer(1/g_sample_dt);   
	g_twist_cmd.angular.z= 0.0;
	g_twist_cmd.linear.x=0.0;
	for (int i=0;i<100;i++) {
		g_twist_commander.publish(g_twist_cmd);
		loop_timer.sleep(); 
	}   
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void PathActionServer::do_spin(double spin_ang) {
	ros::Rate loop_timer(1/g_sample_dt);
	double timer=0.0;
	double final_time = fabs(spin_ang)/g_spin_speed;
	g_twist_cmd.linear.x = 0.2;
	g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
	while(timer<final_time) {
		g_twist_commander.publish(g_twist_cmd);
		timer+=g_sample_dt;
		loop_timer.sleep(); 
	}  
	do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void PathActionServer::do_move(double distance) {
	ros::Rate loop_timer(1/g_sample_dt);
	double timer=0.0;
	double final_time = fabs(distance)/g_move_speed;

    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance) * g_move_speed;
    
    while(timer < final_time) {
    	vel_pub.publish(g_twist_cmd);
    	timer+=g_sample_dt;
    	loop_timer.sleep(); 
    }  
    do_halt();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "PathActionServer_node"); // name this node 

    ROS_INFO("instantiating the PathActionServer: ");

    PathActionServer as_object; // create an instance of the class "PathActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

