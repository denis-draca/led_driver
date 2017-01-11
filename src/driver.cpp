#include "driver.h"

/*
 * led0 -> Led
 * led1 -> Peltier
 *
 * Naming stems from the fact that LED drivers are used to control both led and peltier
 *
 */

Driver::Driver(ros::NodeHandle &n, int pin):
    n_(n), pin_(pin)
{
    value_path = "sys/class/gpio/gpio";
    value_path.append(to_string(pin));
    value_path.append("/value");

    led0 = new led_driver_controller(0, 1000, 0.0);
    led1 = new led_driver_controller(1, 1000, 0.0);

    sub_led0_led0_ = n_.subscribe("/driver_control/led/new_brightness/led0", 1, &Driver::led0_subscriber, this);
    sub_led0_led1_ = n_.subscribe("/driver_control/led/new_brightness/led1", 1, &Driver::led1_subscriber, this);
    sub_led1_peltier_ = n_.subscribe("/driver_control/peltier/new_duty", 1, &Driver::peltier_subscriber, this);

    pub_led_state_ = n_.advertise<std_msgs::Int16>("/driver_control/led/current_brightness", 1);
    pub_peltier_state_ = n_.advertise<std_msgs::Int16>("/driver_control/peltier/current_duty", 1);

    led_state.data = 0;
    peltier_state.data = 0;

    control_file.open("/sys/class/gpio/export");

    if(!control_file.is_open())
    {
        throw "Cannot open pin export file";
    }
    else
    {
        control_file << pin_;
    }

    control_file.close();

    control_file.open("/sys/class/gpio/gpio26/direction");

    if(!control_file.is_open())
    {
        throw "Cannot open direction file";
    }
    else
    {
        control_file << "out";
    }

    control_file.close();

    led0->start();
    led1->start();
}

Driver::~Driver()
{
    std::cout << "\033[1;31m\nCleaning Resources then closing\033[0m" << std::endl;
    delete led0;
    delete led1;

    control_file.open("/sys/class/gpio/unexport");

    control_file << pin_;
}

void Driver::seperateThread()
{
    while(ros::ok())
    {
        std::this_thread::sleep_for(chrono::milliseconds(500));

        pub_led_state_.publish(led_state);
        pub_peltier_state_.publish(peltier_state);
    }
}


void Driver::led0_subscriber(const std_msgs::Int16ConstPtr &input)
{
    if(!first_led_on_)
    {
        control_file.open(value_path.c_str());

        if(!control_file.is_open())
        {
            throw "Cannot Write new pin value";
        }
        else
        {
            control_file << 1;
        }

        first_led_on_ = true;
        second_led_on_ = false;
    }

    control_file.close();

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

    led_state.data = new_data;
}

void Driver::led1_subscriber(const std_msgs::Int16ConstPtr &input)
{
    if(!second_led_on_)
    {
        control_file.open(value_path.c_str());

        if(!control_file.is_open())
        {
            throw "Cannot Write new pin value";
        }
        else
        {
            control_file << 0;
        }

        second_led_on_ = true;
        first_led_on_ = false;
    }

    control_file.close();
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

    led_state.data = new_data;
}

void Driver::peltier_subscriber(const std_msgs::Int16ConstPtr &input)
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
    peltier_state.data = new_data;

}

