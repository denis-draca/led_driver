#include "led_driver_controller.h"

led_driver_controller::led_driver_controller(int pin, int period, double duty)
{
    if (pin >= 2)
    {
        throw std::invalid_argument("pin doesnt exist");
    }
    if (duty > 1.0)
    {
        throw std::invalid_argument("duty out of range");
    }
    else
    {
        _connected_pin = pin;
        _period = period;
        _duty = duty;
    }
}

int led_driver_controller::stop()
{
    ofstream file;
    if(_connected_pin == 0){
        file.open("/sys/class/pwm/pwmchip0/unexport");
    }
    else{
        file.open("/sys/class/pwm/pwmchip1/unexport");
    }

    if(!file.is_open())
    {
        fprintf(stderr, "Failed to unexport pin\n");
    }
    file << _connected_pin;

    file.close();

    std::cout << "\033[1;31m\nPWM Stopped, use start() to start PWM Again\033[0m" << std::endl;
}

led_driver_controller::~led_driver_controller()
{
    stop();
}

int led_driver_controller::start()
{
    if(!start_called_)
    {
        char buffer[BUFFER_MAX];
        ssize_t bytes_written;
        int fd;

        if(_connected_pin == 0){
            fd = open("/sys/class/pwm/pwmchip0/export", O_WRONLY);
        }
        else{
            fd = open("/sys/class/pwm/pwmchip1/export", O_WRONLY);
        }

        if (-1 == fd) {
            fprintf(stderr, "Failed to open export for PWM writing!\n");
            return(-1);
        }

        bytes_written = snprintf(buffer, BUFFER_MAX, "%d", 0);
        write(fd, buffer, bytes_written);
        close(fd);

        ofstream enable_file;
        if (_connected_pin == 0)
        {
            enable_file.open("/sys/class/pwm/pwmchip0/pwm0/enable");
        }
        else
        {
            enable_file.open("/sys/class/pwm/pwmchip1/pwm0/enable");
        }

        if(!enable_file.is_open())
        {
            fprintf(stderr, "Failed to open ENABLE file for writing!\n");
            return -1;
        }

        change_period(_period);
        change_duty(_duty);

        enable_file << 1;
        enable_file.close();

        start_called_ = true;
    }

    else
    {
        std::cout << "\033[1;31m\nPWM Already Running, USE change_period or change_duty to make changes to the PWM value\033[0m" << std::endl;
    }


}

int led_driver_controller::change_period(int new_period)
{
    _period = new_period;

    ofstream period_file;

    if (_connected_pin == 0){
        period_file.open("/sys/class/pwm/pwmchip0/pwm0/period");
    }
    else
    {
        period_file.open("/sys/class/pwm/pwmchip1/pwm0/period");
    }

    if (!period_file.is_open())
    {
        fprintf(stderr, "Failed to open PERIOD file for writing!\n");
        return -1;
    }

    period_file << _period;
}

int led_driver_controller::change_duty(double new_duty)
{
    if (new_duty > 1.0)
    {
        return -1;
    }
    _duty = new_duty;

    ofstream duty_file;

    if (_connected_pin == 0){
        duty_file.open("/sys/class/pwm/pwmchip0/pwm0/duty_cycle");
    }
    else
    {
        duty_file.open("/sys/class/pwm/pwmchip1/pwm0/duty_cycle");
    }

    if (!duty_file.is_open())
    {
        fprintf(stderr, "Failed to open DUTY file for writing!\n");
        return -1;
    }

    duty_file << (_period * _duty);
}
