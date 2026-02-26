#include <stdio.h>
#include <iostream>
#include "rpi_pwm.h"

float angleToDutyCycle(int angle) {

    // Mapping angles for DS with 500us 0° position with 0-180 operation range
    float pulse_us = 500.0f + (angle * 1500.0f / 180.0f);
    // Convert to duty cycle % at 50Hz (20000µs period)
    return (pulse_us / 20000.0f) * 100.0f;
}


int main() {
    int channel_pwm = 2;
    int freq_pwm = 50;
    // Setup PWM channel and control frequency
    printf("Enabling PWM on channel %d.\n",channel_pwm);

    // Define PWM "Device" and start channel
    RPI_PWM servo;
    servo.start(channel_pwm, freq_pwm);
    for (int i = 0; i < 5; i++){
        servo.setDutyCycle(angleToDutyCycle(45));
        //std::cout << "Duty cycle at " << i*5 << "%";
        sleep(3);
        servo.setDutyCycle(angleToDutyCycle(50));
        sleep(1);
        servo.setDutyCycle(angleToDutyCycle(40));
        sleep(1);
        servo.setDutyCycle(angleToDutyCycle(47));
        sleep(1);
    }

    servo.stop();
}