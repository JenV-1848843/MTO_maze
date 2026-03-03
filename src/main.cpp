#include <stdio.h>
#include <iostream>
// Option 1: Add this include at the top of your file
#include <cstdint>
#include "rpi_pwm.h"
#include "pi2c.h"


int main() {
    // Setup PWM channel and control frequency
    int channel_pwm = 2; // Servo aan buitenkant, GPIO 18
    int channel_pwm_y = 3; // Servo aan binnenkant, GPIO 19
    int freq_pwm = 50;
    RPI_PWM servo_x, servo_y;
    servo_x.start(channel_pwm, freq_pwm);
    servo_y.start(channel_pwm_y, freq_pwm);

    //Setup I2C channel
    char adr = 0x68; // standaard adres IMU MPU6050
    //char adr_2 = 0x69; // AD0 pin naar VCC verbinden op 2e IMU
    Pi2c imu(adr);
    char input[10]; // input buffer for I2C input

    // wake up and initialise IMU
    imu.init();
    std::cout << "starting loop" << std::endl;
    // Test loop voor servo bewegingen
    for (int i = -20; i < 20; i++){
        int angle = i;
        int offset_x = 43;
        int offset_y = -43;
        servo_x.setDutyCycle(angleToDutseyCycle(angle, offset_x));
        servo_y.setDutyCycle(angleToDutseyCycle(angle, offset_y));
        std::cout << "Duty cycle at " << angle << "°" << std::endl;
        usleep(200000);
    }

    // Uitlezen van gyroscoop... te verplaatsen naar een functie/klasse
    bool gyro_connected = false;
    if (gyro_connected){

        imu.i2cRead(input, 10);
        std::cout << "received: " << input << std::endl;
        char gyro_1[6];
        imu.readGyro(gyro_1);
        // Combine high and low bytes into 16-bit signed integers
        int16_t gyro_x = (int16_t)((uint8_t)gyro_1[0] << 8 | (uint8_t)gyro_1[1]);
        int16_t gyro_y = (int16_t)((uint8_t)gyro_1[2] << 8 | (uint8_t)gyro_1[3]);
        int16_t gyro_z = (int16_t)((uint8_t)gyro_1[4] << 8 | (uint8_t)gyro_1[5]);
        
        // Convert raw values to degrees/sec (±250°/s range → 131 LSB per °/s)
        float gx = gyro_x / 131.0f;
        float gy = gyro_y / 131.0f;
        float gz = gyro_z / 131.0f;
        
        std::cout << "Gyro X: " << gx << " °" << std::endl;
        std::cout << "Gyro Y: " << gy << " °" << std::endl;
        std::cout << "Gyro Z: " << gz << " °" << std::endl;
    }
        
   servo_x.stop();
   servo_y.stop();
}
