#include <shell_simulation/example_publisher.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "shell_simulation");
    
    ExamplePublisher node;

    ros::Rate rate(10);

    int count = 0;
    const int max_count = 100;

    while (ros::ok() && count < max_count) {
        node.publish();
        ros::spinOnce();
        rate.sleep();
        count++;
    }

    return 0;
}
