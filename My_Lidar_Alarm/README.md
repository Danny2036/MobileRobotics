# My_Lidar_Alarm

A simulation of a robot running through a course. The robot it made to send a sensor out to ping for obstacles. Initially there was a single  ping directly in front of the robot. This was updated to ping a sweep in front of the robot to account for the robot width and for more than the obstacles directly in front. There is an alarm set on the robot to sound when an obstacle does appear. The reactive commander is used to turn the robot to avoid an on comming obstacle and turn

## Example usage

Run the maze and robot using: roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch 

Activate the alarm using: rosrun My_Lidar_Alarm My_Lidar_Alarm

Activate the reactive commander using: rosrun stdr_control reactive_commander

## Running tests/demos
    
A video of the robot workign successfully is attached in the zip folder
