#include "ros/ros.h"
#include "driver.h"
//#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "led_driver_controller");
    ros::NodeHandle n;


    std::shared_ptr<Driver> gc(new Driver(n));
    std::thread t(&Driver::seperateThread,gc);

    //  Driver driver(n);

    ros::spin();
    ros::shutdown();

    t.join();

    return 0;
}
