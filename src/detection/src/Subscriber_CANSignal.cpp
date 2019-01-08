#include "ros/ros.h"
#include <vector>
#include "Subscriber_CANSignal.cpp"
#include "boost/tuple"
#include <ros/node_handle.h>

using namespace std;
using boost::tuple;

bool Subscriber::Init(ros::NodeHandle& n)
{
	nh_ = n;
    yaml_subscriber_ = nh_.subscribe("topic_anomalySignals",
                                     1000,
                                     &SubscriberCallback);
	return true; 
}


int Subscriber::Execute()
{
    if (!initialized_)
    {
        ROS_ERROR("SRR4 Publisher not initialized, please call Init before Execute");
        return 1;
    }
}

void Subscriber::SubscriberCallback(const input::anomalySignals& msg)
{
	typedef vector< tuple<uint,double,double,bool,bool,bool,bool,uint>> tuple_list;
	tuple_list Subscriber::ReadCANSignal(const input::anomalySignals& msg){
	tuple_list tl;
	for (tuple_list::const_iterator i = tl.begin(); i != tl.end(); ++i) {
		
		ROS_INFO(" msgId : [%u],  vehicleSpeed : [%lf], 
				 engineSpeed : [%lf], 
				 driverdoor_open : [%d], 
				 frontPassengerDoor_open : [%d], 
				 rearLeftDoor_open : [%d], 
				 rearRightDoor_open : [%d] and
				 frameTickCountSysReadTime : [%u]",
				 i->get<0>(),
				 i->get<1>(),
				 i->get<2>(),
				 i->get<3>(),
				 i->get<4>(),
				 i->get<5>(),
				 i->get<6>(),
				 i->get<7>())
	    }
   }
}