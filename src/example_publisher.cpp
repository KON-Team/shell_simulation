#include <shell_simulation/example_publisher.h>

#include <std_msgs/String.h>

ExamplePublisher::ExamplePublisher() {
   string_pub_ = nh_.advertise<std_msgs::String>("string", 1);
}

void ExamplePublisher::publish() {
    std_msgs::String msg;
    msg.data = "Hello";
    string_pub_.publish(msg);
}
