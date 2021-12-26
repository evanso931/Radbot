#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <iostream>

/** struct for points **/
struct navPoints {
	double x; // x coordinates
	double y; // y coordinates
};

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal);
int count_lines();
void read_into_array(navPoints array[], int nLines);

bool goalReached = false;

int main(int argc, char** argv){
	// do csv operations
	int nLines = count_lines(); // count how many lines
	navPoints *points = new navPoints[nLines]; // create dynamic array to store csv data
	read_into_array(points, nLines); // read csv into array

	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();
	// tell the action client that we want to spin a thread by default

	for(int i = 1; i < nLines+1; i++){
		goalReached = moveToGoal(points[i].x, points[i].y);
	}
	
	return 0;
}

// function from https://edu.gaitech.hk/turtlebot/map-navigation.html
bool moveToGoal(double xGoal, double yGoal){

	// define a client to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

}

int count_lines() {
  // create an input file stream
  std::ifstream input;
  // open the csv of points
  input.open("nav_points.csv");
  // check if the file is not open
  if (!input.is_open()) {
    // print error message and quit if a problem occurred
    std::cerr << "Error! No input file found!\n";
    exit(1);
  }
  int n = 0;
  std::string dummy;
  // keep reading lines in file until no lines left to read
  // read into dummy string and increment count
  while (getline(input, dummy)) {
    n++;
  }
  return n;
}

void read_into_array(navPoints array[], int n) {
  // create an input file stream
  std::ifstream input;
  // open the csv of points
  input.open("nav_points.csv");
  // check if the file is not open
  if (!input.is_open()) {
    // print error message and quit if a problem occurred
    std::cerr << "Error! No input file found!\n";
    exit(1);
  }
  std::string dummy;
  // loop through each line in file
  for (int i = 0; i < n; i++) {
    getline(input, dummy, ',');  // read until first comma, so gets the x coord
    array[i].x = std::stoi(dummy); // convert to integer and store in x of struct
    getline(input, dummy);  // read up to end of line, so gets the y coord
    array[i].y = std::stoi(dummy); // convert to integer and store in y of struct
  }
}