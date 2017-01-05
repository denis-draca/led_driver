#include "driver.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "led_driver_controller");
    ros::NodeHandle n;

    std::shared_ptr<Driver> gc(new Driver(n));
    std::thread t(&Driver::seperateThread,gc);
    ros::spin();
    ros::shutdown();

    t.join();

    return 0;
}
