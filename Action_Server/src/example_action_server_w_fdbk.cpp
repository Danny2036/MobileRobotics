// Action_Server: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../Action_Server/action/Action.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (Action) and appended name (Action)
#include<Action_Server/gazebo.action.h>

int g_count = 0;
bool g_count_failure = false;

class ExampleActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in Action_Server/action/Action.action
    // the type "gazebo.action" is auto-generated from our name "Action" and generic name "Action"
    actionlib::SimpleActionServer<Action_Server::gazebo.action> as_;
    
    // here are some message types to communicate with our client(s)
    Action_Server::ActionGoal goal_; // goal message, received from client
    Action_Server::ActionResult result_; // put results here, to be sent back to the client when done w/ goal
    Action_Server::ActionFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int countdown_val_;


  public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<Action_Server::gazebo.action>::GoalConstPtr& goal);
  };

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class exampleActionServer.  
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

  ExampleActionServer::ExampleActionServer() :
  as_(nh_, "timer_action", boost::bind(&ExampleActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
  {
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
  }

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <Action_Server::gazebo.action> customizes the simple action server to use our own "action" message 
// defined in our package, "Action_Server", in the subdirectory "action", called "Action.action"
// The name "Action" is prepended to other message types created automatically during compilation.
// e.g.,  "gazebo.action" is auto-generated from (our) base name "Action" and generic name "Action"
  void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<Action_Server::gazebo.action>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes
    ros::Rate timer(1.0); // 1Hz timer
    countdown_val_ = goal->input;
    //implement a simple timer, which counts down from provided countdown_val to 0, in seconds
    while (countdown_val_>0) {
     ROS_INFO("countdown = %d",countdown_val_);
     
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

  int main(int argc, char** argv) {
    ros::init(argc, argv, "timer_action_server_node"); // name this node 

    ROS_INFO("instantiating the timer_action_server: ");

    ExampleActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
  }

