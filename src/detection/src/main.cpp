#include "ros/ros.h"
#include "header_file.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Anamoly_CANSignal_node");
	ros::NodeHandle nh; 
	
	ros::Subscriber subscriber; 
	    if (!subscriber.Init(nh))
    {
        // Initialization was unsuccessful
        ROS_ERROR("\n%s: Error during parameter initialization: %s \n",);
        return 1;
    }
}
	
	
	
	