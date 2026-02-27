#include <stdio.h>
#include <iostream>
#include "rpi_pwm.h"`
#include "pi2c.h"


int main() {
    // Setup PWM channel and control frequency
    int channel_pwm = 2;
    int freq_pwm = 50;
    printf("Enabling PWM on channel %d.\n",channel_pwm);
    RPI_PWM servo;
    servo.start(channel_pwm, freq_pwm);

    //Setup I2C channel
    int adr = 104; // standaard adres IMU MPU6050
    int adr_2 = 105; // AD0 pin naar VCC verbinden op 2e IMU
    Pi2c imu(adr);
    char input[10]; // input buffer for I2C input

    for (int i = 0; i < 3; i++){
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

    imu.i2cRead(input, 10);
    std::cout << "received: " << input << std::endl;

    servo.stop();
}