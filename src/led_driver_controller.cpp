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

led_driver_controller::~led_driver_controller()
{
    ofstream file;
    if(_connected_pin == 0){
        file.open("/sys/class/pwm/pwmchip0/unexport");
    }
    else{
        file.open("/sys/class/pwm/pwmchip1/unexport");
    }

    file << _connected_pin;

    file.close();
}

int led_driver_controller::start()
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

    ofstream period_file;
    ofstream duty_file;
    ofstream enable_file;
    if (_connected_pin == 0){
        period_file.open("/sys/class/pwm/pwmchip0/pwm0/period");
        duty_file.open("/sys/class/pwm/pwmchip0/pwm0/duty_cycle");
        enable_file.open("/sys/class/pwm/pwmchip0/pwm0/enable");
    }
    else{
        period_file.open("/sys/class/pwm/pwmchip1/pwm0/period");
        duty_file.open("/sys/class/pwm/pwmchip1/pwm0/duty_cycle");
        enable_file.open("/sys/class/pwm/pwmchip1/pwm0/enable");
    }

    if(!period_file.is_open())
    {
        fprintf(stderr, "Failed to open PERIOD file for writing!\n");
        return -1;
    }

    if(!duty_file.is_open())
    {
        fprintf(stderr, "Failed to open DUTY file for writing!\n");
        return -1;
    }

    if(!enable_file.is_open())
    {
        fprintf(stderr, "Failed to open ENABLE file for writing!\n");
        return -1;
    }

    period_file << _period;
    period_file.close();

    duty_file << _period*_duty;
    duty_file.close();

    enable_file << 1;
    enable_file.close();


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
