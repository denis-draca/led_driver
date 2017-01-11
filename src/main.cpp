#include "driver.h"

int main(int argc, char **argv)
{
    int pin = 25;

    std::string str;

    if(argc > 1)
    {
        str = argv[1];

        if(str.size() > 2)
        {
            ROS_ERROR("Value Input too Large, should be 2 characters at most");
            return 0;
        }
        else
        {
            try
            {
                int new_pin = stoi(str);
                pin = new_pin;
            }
            catch(const std::exception &e)
            {
                ROS_ERROR("Invalid Input, Exception thrown: %s", e.what());
                return 0;
            }
        }
    }

    ROS_INFO("Using Pin for switching LED's: %d", (int)pin);

    ros::init(argc, argv, "led_driver_controller");
    ros::NodeHandle n;

    try
    {
        std::shared_ptr<Driver> gc(new Driver(n,pin));
        std::thread t(&Driver::seperateThread,gc);
        ros::spin();


        t.join();
    }
    catch(const char* s)
    {
        ROS_ERROR("PROBLEM: %s", s);
    }

    ros::shutdown();

    return 0;
}
