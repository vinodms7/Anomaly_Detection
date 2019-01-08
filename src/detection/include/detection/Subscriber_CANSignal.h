#ifndef SUBSCRIBER_CANSIGNAL_H
#define SUBSCRIBER_CANSIGNAL_H



class Subscriber
{
  public:
    Subscriber();
    ~Subscriber();
	
  	 bool Init(ros::NodeHandle& n);
  	 int Execute();
};
#endif //SUBSCRIBER_CANSIGNAL_H
