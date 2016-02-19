// Action_Server: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../Action_Server/action/Action.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (Action) and appended name (Action)
#include<Action_Server/path_msg.action.h>

geometry_msgs::Twist g_twist;
ros::Publisher g_twist_commander;
geometry_msgs::Pose g_pose;

class PathActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in Action_Server/action/Action.action
    // the type "path_msg.action" is auto-generated from our name "Action" and generic name "Action"
    actionlib::SimpleActionServer<Action_Server::path_msg.action> as_;
    ros::Publisher vel_pub; //Publisher that published to cmd_
    
    // here are some message types to communicate with our client(s)
    Action_Server::ActionGoal goal_; // goal message, received from client
    Action_Server::ActionResult result_; // put results here, to be sent back to the client when done w/ goal
    Action_Server::ActionFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int countdown_val_;


  public:
    PathActionServer(); //define the body of the constructor outside of class definition

    ~PathActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<Action_Server::path_msg.action>::GoalConstPtr& goal);

    double sgn(double x);
    double min_spin(double spin_angle);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

    void do_inits(ros::NodeHandle &n);
    void do_halt();
    void do_spin(double spin_ang);
    void do_move(double distance);
  };

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class PathActionServer.  
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

  PathActionServer::PathActionServer() :
  as_(nh_, "timer_action", boost::bind(&PathActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
  {
    ROS_INFO("in constructor of PathActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
  }

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <Action_Server::path_msg.action> customizes the simple action server to use our own "action" message 
// defined in our package, "Action_Server", in the subdirectory "action", called "Action.action"
// The name "Action" is prepended to other message types created automatically during compilation.
// e.g.,  "path_msg.action" is auto-generated from (our) base name "Action" and generic name "Action"
  void PathActionServer::executeCB(const actionlib::SimpleActionServer<Action_Server::path_msg.action>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    ROS_INFO("goal input is: %d", goal->input);

    std::vector<double> spin_angle = goal->angle;
    std::vector<double> travel_distance = goal->distance;
    int num_angles = spin_angle.size();

    ros::Rate timer(1.0); // 1Hz timer
    countdown_val_ = goal->input;
    //implement a simple timer, which counts down from provided countdown_val to 0, in seconds
    while (countdown_val_>0) {
     ROS_INFO("countdown = %d",countdown_val_);
     
     feedback_.fdbk = countdown_val_;
     as.publishFeedback(feedback_);
     for(int i = 0 ; i < num_angles; i++){
      do_spin(spin_angle[i]);
      do_move(travel_distance[i]);
      ROS_INFO("spin_angle = %f", spin_angle[i]);
      ROS_INFO("travel_distance = %f", travel_distance[i]);
    }

    do_halt();

    countdown_val_--;
    timer.sleep();
  }

  result_.output = countdown_val_;
  as_.setSucceeded(result_);
}


       // each iteration, check if cancellation has been ordered
if (as_.isPreemptRequested()){	
  ROS_WARN("goal cancelled!");
  result_.output = countdown_val_;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
        }
        
 	   //if here, then goal is still valid; provide some feedback
 	   feedback_.fdbk = countdown_val_; // populate feedback message with current countdown value
 	   as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
       countdown_val_--; //decrement the timer countdown
       timer.sleep(); //wait 1 sec between loop iterations of this timer
     }
    //if we survive to here, then the goal was successfully accomplished; inform the client
    result_.output = countdown_val_; //value should be zero, if completed countdown
    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
  }

  /signum function: strip off and return the sign of the argument
  double MobotActionServer::sgn(double x) {
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
double MobotActionServer::min_spin(double spin_angle) {
  if (spin_angle > M_PI) {
    spin_angle -= 2.0*M_PI;}
    if (spin_angle < -M_PI) {
      spin_angle += 2.0*M_PI;}
      return spin_angle;   
    }            

// a useful conversion function: from quaternion to yaw
    double MobotActionServer::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
      double quat_z = quaternion.z;
      double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
  }

//and the other direction:
  geometry_msgs::Quaternion MobotActionServer::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
  }

  void MobotActionServer::do_inits(ros::NodeHandle &n) {
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
    
    // we declared g_twist_commander as global, but never set it up; do that now that we have a node handle
    g_twist_commander = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);    
  }

  void MobotActionServer::do_halt() {
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
  void MobotActionServer::do_spin(double spin_ang) {
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
void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

  int main(int argc, char** argv) {
    ros::init(argc, argv, "timer_action_server_node"); // name this node 

    ROS_INFO("instantiating the timer_action_server: ");

    PathActionServer as_object; // create an instance of the class "PathActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
  }

