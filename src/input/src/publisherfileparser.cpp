#include "ros/ros.h"
#include "publisherfilepareser.h"
#include <yaml-cpp/yaml.h>


void PublisherFileParser::HandleCANSignalEntry(const YAML::Node& yaml_entry)
{
	using namespace fileparser;
	
	const YAML::Node& yaml_cansignal_entry = yaml_entry["cansignal"];
	for( YAML::const_iterator it = yaml_cansignal_entry.begin(); it != yaml_cansignal_entry.end(); it++) 
	{
		const YAML::Node& cansignal_entry = *it;
		
		cansignal_entry.msgId = GetKeyValue<unsigned int>(msgId, "msgId");
		cansignal_entry.vehicleSpeed = GetKeyValue<double>(vehicleSpeed, "vehicleSpeed");
		cansignal_entry.engineSpeed = GetKeyValue<double>(engineSpeed,"engineSpeed");
		cansignal_entry.driverdoor_open = GetKeyValue<bool>(driverdoor_open,"driverdoor_open");
		cansignal_entry.frontPassengerDoor_open = GetKeyValue<bool>(frontPassengerDoor_open,"frontPassengerDoor_open");
		cansignal_entry.rearLeftDoor_open = GetKeyValue<bool>(rearLeftDoor_open,"rearLeftDoor_open");
		cansignal_entry.rearRightDoor_open = GetKeyValue<bool>(rearRightDoor_open,"rearRightDoor_open");
		cansignal_entry.frameTickCountSysReadTime = GetKeyValue<unsigned int>(frameTickCountSysReadTime,"frameTickCountSysReadTime");
		
		ros::ROS_INFO(" msgId : [%u],  vehicleSpeed : [%lf], 
				 engineSpeed : [%lf], 
				 driverdoor_open : [%d], 
				 frontPassengerDoor_open : [%d], 
				 rearLeftDoor_open : [%d], 
				 rearRightDoor_open : [%d] and
				 frameTickCountSysReadTime : [%u]",
				 cansignal_entry.msgId,
				 cansignal_entry.vehicleSpeed,
				 cansignal_entry.engineSpeed,
				 cansignal_entry.driverdoor_open, 
				 cansignal_entry.frontPassengerDoor_open, 
				 cansignal_entry.rearLeftDoor_open, 
				 cansignal_entry.rearRightDoor_open, 
				 cansignal_entry.frameTickCountSysReadTime);
			
		ros_canSignal_data_ptr_->anomalySignals.push_back(cansignal_entry);
	}
}

PublisherFileParser::PublisherFileParser(ros::NodeHandle nh) : node_handle_(nh), rate_(10)
{
    ros_canSignal_data_ptr_ = boost::make_shared<input::anomalySignals>();	
}
			
void PublisherFileParser::LoadAndGenerateMessages()
{
	
	if (!ReadYamlData())
    {
        ROS_INFO_STREAM(" YAML file not load");
        return;
    }

    if (!GenerateRosMsg())
    {
        ROS_INFO_STREAM("Error in converting the yaml-file to the ros-msg");
        return;
    }		
	
	//pub_ = node_handle_.advertise<input::anomalySignals>("topic_anomalySignals", 1);
	 pub_ = node_handle_.advertise<bmw_sen_msgs::SensorCalibrationDataList>(topic_name_out_, 1);
}

input::anomalySignals PublisherFileParser::GetCANSignalData()
{
	return *ros_canSignal_data_ptr_
}

bool PublisherFileParser::GetRosParams()
{
	 if (!node_handle_.getParam("output_file_path", file_path_))  //yaml file i/p path
	 {
		 ros::ROS_INFO_STREAM("Invalid file path");
		 return false;
	 }
	
	if (!node_handle_.getParam("calibration_data_out", topic_name_out_))
    {
        ROS_INFO_STREAM("[VIGEMParser] Invalid topic name");
        return false;
    }
	
	
	int32_t freq = 0;
    node_handle_.param<int32_t>("frequency", freq, 45);
    rate_ = ros::Rate(freq);
	
	return true;
}

void PublisherFileParser::StartPublishing()
{
    ROS_INFO_STREAM(" Start spinning ... ");
    while (ros::ok())
    {
        PublishData();
        ros::spinOnce();
        rate_.sleep();
    }
}			

void PublisherFileParser::Init()
{
    if (!GetRosParams())
    {
        throw std::runtime_error("Invalid ros params");
    }
  
    ROS_INFO_STREAM("YAML File path : " << file_path_);  
	ROS_INFO_STREAM("Calibration data topic (out): " << topic_name_out_);
// ROS_INFO_STREAM(" Rate (hz): " << 1. / rate_.expectedCycleTime().toSec());
}

bool PublisherFileParser::ReadYamlData()
{
    if (boost::filesystem::exists(file_path_))
    {
        yaml_cansignal_data_ = YAML::LoadFile(file_path_);
        if (!yaml_cansignal_data_)
            return false;
        return true;
    }
    else
    {
        return false;
    }
}

bool PublisherFileParser::HandleSignalEntry(const YAML::Node& yaml_entry)
{
    using namespace fileparser;

    HandleCAnSignalEntry(yaml_entry);
    return true;
}

void PublisherFileParser::PublishData()
{
    ros_canSignal_data_ptr_->header.stamp = ros::Time::now();
    pub_.publish(ros_canSignal_data_ptr_);
}

































