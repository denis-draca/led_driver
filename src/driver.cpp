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
}


void Driver::led0_subscriber(const std_msgs::Int16ConstPtr &input)
{
    led0->start();

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
}

void Driver::led1_subscriber(const std_msgs::Int16ConstPtr &input)
{

    led1->start();

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

}

