#include "ros/ros.h"
#include "driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "led_driver_controller");
  ros::NodeHandle n;

  Driver driver(n);

  ros::spin();
  ros::shutdown();

  return 0;
}
