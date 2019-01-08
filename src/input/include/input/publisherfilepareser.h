#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

class PublisherFileParser
{
  public:
    PublisherFileParser() = delete;
    PublisherFileParser(ros::NodeHandle nh);

    void Init();
    void LoadAndGenerateMessages();
    void StartPublishing();

    input::anomalySignals GetCANSignalData();
    bool ReadYamlData();
    
  private:
    YAML::Node yaml_calibration_data_;
    input::anomalySignals ros_canSignal_data_ptr_;
    ros::NodeHandle node_handle_;
    ros::Rate rate_;
    std::string file_path_;
    std::string calibration_file_pattern_;
    ros::Publisher pub_;
    std::string topic_name_out_;
    

    void PublishData();    
    bool GetRosParams();    
	bool HandleSignalEntry(const YAML::Node& yaml_entry);
    void HandleCANSignalEntry(const YAML::Node& yaml_entry);
};
    