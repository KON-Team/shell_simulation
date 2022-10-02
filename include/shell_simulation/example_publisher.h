#include <ros/ros.h>

class ExamplePublisher {
  public:
    ExamplePublisher();

    void publish();

  private:
    ros::NodeHandle nh_;

    ros::Publisher string_pub_;
};
