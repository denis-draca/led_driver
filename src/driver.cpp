#include "driver.h"

Driver::Driver(ros::NodeHandle &n):
    n_(n)
{
    led0 = new led_driver_controller(0, 1000, 0.0);
    led1 = new led_driver_controller(1, 1000, 0.0);

    sub_led0_ = n_.subscribe("/driver_control/new_brightness/led0", 1, &Driver::led0_subscriber, this);
    sub_led1_ = n_.subscribe("/driver_control/new_brightness/led1", 1, &Driver::led1_subscriber, this);

    pub_led0_state_ = n_.advertise<std_msgs::Int16>("/driver_control/current_brightness/led0", 1);
    pub_led1_state_ = n_.advertise<std_msgs::Int16>("/driver_control/current_brightness/led1", 1);

    led0_state.data = 0;
    led1_state.data = 0;

    led0->start();
    led1->start();
}

Driver::~Driver()
{
    std::cout << "\033[1;31m\nCleaning Resources then closing\033[0m" << std::endl;
    delete led0;
    delete led1;
}

void Driver::seperateThread()
{
    while(ros::ok())
    {
        std::this_thread::sleep_for(chrono::milliseconds(500));

        pub_led0_state_.publish(led0_state);
        pub_led1_state_.publish(led1_state);
    }
}


void Driver::led0_subscriber(const std_msgs::Int16ConstPtr &input)
{
    int new_data = input->data;

    if (new_data < 0)
    {
        new_data = 0;
    }

    if (new_data > 100)
    {
        new_data = 100;
    }

    double fraction = (double)new_data/100.0;

    led0->change_duty(fraction);

    led0_state.data = new_data;
}

void Driver::led1_subscriber(const std_msgs::Int16ConstPtr &input)
{
    int new_data = input->data;

    if (new_data < 0)
    {
        new_data = 0;
    }

    if (new_data > 100)
    {
        new_data = 100;
    }

    double fraction = (double)new_data/100.0;

    led1->change_duty(fraction);
    led1_state.data = new_data;

}

