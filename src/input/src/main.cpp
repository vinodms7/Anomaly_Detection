#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Input_CANSignal_node");
    ros::NodeHandle node;

    PublisherFileParser publisher_file_parser(node);

    publisher_file_parser.Init();
    publisher_file_parser.LoadAndGenerateMessages();
    publisher_file_parser.StartPublishing();

    return 0;
}