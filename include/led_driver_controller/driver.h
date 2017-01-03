#ifndef DRIVER_H
#define DRIVER_H

#include "ros/ros.h"
//#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include "led_driver_controller.h"
#include <iostream>
#include <thread>
class Driver
{
private:
    ros::NodeHandle n_;

    ros::Subscriber sub_led0_;
    ros::Subscriber sub_led1_;

    ros::Publisher pub_led0_state_;
    ros::Publisher pub_led1_state_;

    led_driver_controller *led0;
    led_driver_controller *led1;

    std_msgs::Int16 led0_state;
    std_msgs::Int16 led1_state;

private:
    void led0_subscriber(const std_msgs::Int16ConstPtr &input);
    void led1_subscriber(const std_msgs::Int16ConstPtr &input);

    Driver();
public:
    Driver(ros::NodeHandle &n);
    void seperateThread();
};

#endif // DRIVER_H
