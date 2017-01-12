#ifndef DRIVER_H
#define DRIVER_H

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "led_driver_controller.h"
//#include <iostream>
#include <thread>
class Driver
{
private:
    //Nodehandle
    ros::NodeHandle n_;

    //Subscribers
    ros::Subscriber sub_led0_led0_;
    ros::Subscriber sub_led0_led1_;
    ros::Subscriber sub_led1_peltier_;

    //Publishers
    ros::Publisher pub_led_state_;
    ros::Publisher pub_peltier_state_;

    //LED Contoller objects
    led_driver_controller *led0;
    led_driver_controller *led1;

    //Publisher Messages
    std_msgs::Int16 led_state;
    std_msgs::Int16 peltier_state;

    bool first_led_on_ = false;
    bool second_led_on_ = false;

    int pin_;

    std::ofstream control_file;

    std::string value_path;

private:
    ///
    /// \brief led0_subscriber - Takes a message input and changes the brightness to the value given
    /// \param(std_msgs::Int16) input - A percentage value from 0-100, any other numbers will be clipped to fit
    ///
    void led0_subscriber(const std_msgs::Int16ConstPtr &input);

    void led1_subscriber(const std_msgs::Int16ConstPtr &input);

    ///
    /// \brief peltier_subscriber - Takes a message input and changes the peltier power to value given
    /// \param(std_msgs::Int16) input - A percentage value from 0-100, any other numbers will be clipped to fit
    ///
    void peltier_subscriber(const std_msgs::Int16ConstPtr &input);

    Driver();
public:
    ~Driver();//!< Cleans up the pointers built within this class (The LED Driver Controllers) which in turn will call the objects destructor causing a system clean up for sysfs

    Driver(ros::NodeHandle &n, int pin);

    ///
    /// \brief seperateThread - Method running in its own thread publishing the current brightness of the LED's (As a percentage of max), the message is sent every 500ms (2Hz)
    ///
    void seperateThread();
};

#endif // DRIVER_H
