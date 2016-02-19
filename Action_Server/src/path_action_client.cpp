// path_action_client: 

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_action_server.h>
#include<nav_msgs/Path.h>

//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../Action_Server/action/path_msg.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (path_msg) and appended name (Action)
// If you write a new client of the server in this package, you will need to include Action_Server in your package.xml,
// and include the header file below
#include<Action_Server/path_msg.action.h>

bool lidar_lidar_active =false;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void alarmCallback (const actionlib::SimpleClientGoalState& state, const Action_Server::path_msgResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    int diff = result->output - result->goal_stamp;
    ROS_INFO("got result output = %d; goal_stamp = %d; diff = %d",result->output,result->goal_stamp,diff);
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "path_msg_action_client_node"); // name this node 
        int g_count = 0;
        // here is a "goal" object compatible with the server, as defined in Action_Server/action
        Action_Server::path_msgGoal goal; 
        
        // use the name of our server, which is: path_action (named in Action_Server.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        actionlib::SimpleActionClient<Action_Server::path_msg.action> action_client("path_action", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        
        
        ROS_INFO("connected to action server");  // if here, then we connected to the server;



        while(ros::ok) {
        // stuff a goal message:
            g_count++;
        goal.input = g_count; // this merely sequentially numbers the goals sent
        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d",g_count);
            return 0;
        }
        else {
          //if here, then server returned a result to us
        }
        
    }

    return 0;
}

