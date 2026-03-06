#include <stdio.h>
#include <iostream>
// Option 1: Add this include at the top of your file
#include <cstdint>
#include "rpi_pwm.h"
#include "pi2c.h"
#include <array>


// Voor key press detectie
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Helper function: check if a key has been pressed (non-blocking)
bool keyPressed(char target) {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);  // Disable buffered input & echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Set stdin to non-blocking
    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    char ch = 0;
    read(STDIN_FILENO, &ch, 1);

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return ch == target;
}



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
    // wake up and initialise IMU
    imu.init();

    
    // Test loop voor servo bewegingen
    bool servo_loop = false;
    if (servo_loop){
        std::cout << "starting loop" << std::endl;
        for (int i = -20; i < 20; i++){
            int angle = i;
            int offset_x = 43;
            int offset_y = -43;
            servo_x.setDutyCycle(angleToDutyCycle(angle, offset_x));
            servo_y.setDutyCycle(angleToDutyCycle(angle, offset_y));
            std::cout << "Duty cycle at " << angle << "°" << std::endl;
            usleep(200000);
        }
    }

    // Uitlezen van gyroscoop... 
    bool gyro_connected = false;
    while (!keyPressed('x')) {
        if (gyro_connected) {
            std::cout << std::endl << "Gyro data: " << std::endl;
            
            auto gyro_1 = imu.readGyro();

            std::cout << "Gyro X: " << gyro_1[0]<< " °/s" << std::endl;
            std::cout << "Gyro Y: " << gyro_1[1] << " °/s" << std::endl;
            std::cout << "Gyro Z: " << gyro_1[2] << " °/s" << std::endl << std::endl;

            usleep(100000);
        } 
        else {
            std::cout << "No gyro, skipping" << std::endl;
            break;  // Exit loop if no gyro connected
        }
    }

    bool read_accel = true;
    if (read_accel){
        float alpha = 0.1f;  // 0.0 = ignore new data, 1.0 = ignore history
        float roll_filtered  = 0.0f;
        float pitch_filtered = 0.0f;


        // Circular buffer for last 5 readings
        const int BUF_SIZE = 5;
        float roll_buf[BUF_SIZE]  = {0};
        float pitch_buf[BUF_SIZE] = {0};
        int buf_index = 0;
        // init 

        auto accel_0 = imu.readAccel();
        float x_0 = accel_0[0];
        float y_0 = accel_0[1];
        int offset_x = 43;
        int offset_y = -43;

        while (!(keyPressed('x'))){
            std::cout << std::endl << "Accelerometer data: " << std::endl;
            auto accel_1 = imu.readAccel();

            roll_filtered  = alpha * accel_1[0] + (1.0f - alpha) * roll_filtered;
            pitch_filtered = alpha * accel_1[1] + (1.0f - alpha) * pitch_filtered;

            // Store in circular buffer
            roll_buf[buf_index]  = roll_filtered;
            pitch_buf[buf_index] = pitch_filtered;
            buf_index = (buf_index + 1) % BUF_SIZE;

        // Average the buffer
        float roll_avg = 0, pitch_avg = 0;
        for (int i = 0; i < BUF_SIZE; i++){
            roll_avg  += roll_buf[i];
            pitch_avg += pitch_buf[i];
        }
        roll_avg  /= BUF_SIZE;
        pitch_avg /= BUF_SIZE;


            int x_m = roll_filtered - x_0;
            int y_m = pitch_filtered - y_0;

            std::cout << "roll: " << x_m<< " °" << std::endl;
            std::cout << "pitch: " << y_m << " °" << std::endl;

            std::cout << "raw data: " << std::endl;
            std::cout << "fax: " << accel_1[2] << " °" << std::endl;
            std::cout << "fay: " << accel_1[3] << " °" << std::endl;
            std::cout << "fay: " << accel_1[4] << " °" << std::endl;

            
            if (x_m < 50) servo_x.setDutyCycle(angleToDutyCycle(x_m/2, offset_x));
            else servo_x.setDutyCycle(angleToDutyCycle(25, offset_x));

            if (y_m < 50) servo_y.setDutyCycle(angleToDutyCycle(y_m/2, offset_y));
            else servo_y.setDutyCycle(angleToDutyCycle(25, offset_y));

            usleep(10000);
        }
    }




        
   servo_x.stop();
   servo_y.stop();
}
