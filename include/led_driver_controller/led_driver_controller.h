#ifndef LED_DRIVER_CONTROLLER_H
#define LED_DRIVER_CONTROLLER_H

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fstream>
#include <stdexcept>

#define BUFFER_MAX 3

using namespace std;

class led_driver_controller
{
private:
    int _connected_pin; //!< Which pin the led driver is connected to, it can be either adc0 or adc1
    int _period;        //!< The period of the adc output
    double _duty;       //!< Duty cycle of the adc output expressed as a percentage of on time

    led_driver_controller();
public:

    ///
    /// \brief start - starts the adc output
    /// \return (int) - returns -1 if the process failed
    ///
    int start();
    ///
    /// \brief change_period - changes the adc period to the new value
    /// \param (int) new_period - new period
    /// \return (int) - returns -1 if the process failed
    ///
    int change_period(int new_period);
    ///
    /// \brief change_duty - changes the duty percentage to the new input
    /// \param (double) new_duty - new duty percentage, expressed from 0.0 to 1.0, all other numbers will be rejected and will be treated as a fail
    /// \return (int) - returns -1 if the process failed
    ///
    int change_duty(double new_duty);
    ///
    /// \brief led_driver_controller
    /// \param pin
    /// \param period
    /// \param duty
    ///
    led_driver_controller(int pin, int period, double duty);

    ~led_driver_controller();   //!< Destructor, cleans up the sys files associated with the pwm pin, it writes to the unexport file

};

#endif // LED_DRIVER_CONTROLLER_H
