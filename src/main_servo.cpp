#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "motorcontrol.h"

/* Convert angle (0–180) to pulse width in microseconds
   Typical servo: 1000us (0°) to 2000us (180°) */
int angle_to_pulse(int angle) {
    return 1000 + (angle * 1000) / 180;
}

/* Set terminal to non-blocking mode */
void set_nonblocking_input(int enable) {
    static struct termios oldt, newt;

    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        newt.c_lflag &= ~(ICANON | ECHO); // no buffering, no echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}

int main() {
    int servo_x_angle = 90;
    int servo_y_angle = 90;
    char ch;

    printf("Initializing GPIO and Servo control...\n");

    Init_GPIO_Control();
    Init_ServoMotor_Control();

    set_nonblocking_input(1);

    printf("Controls:\n");
    printf("  l/m -> Servo X (-/+)\n");
    printf("  o/p -> Servo Y (-/+)\n");
    printf("  q   -> quit\n");

    while (1) {
        ch = getchar();

        if (ch != EOF) {
            switch (ch) {
                case 'l':
                    servo_x_angle--;
                    break;
                case 'm':
                    servo_x_angle++;
                    break;
                case 'o':
                    servo_y_angle--;
                    break;
                case 'p':
                    servo_y_angle++;
                    break;
                case 'q':
                    printf("Exiting...\n");
                    set_nonblocking_input(0);
                    Exit_Motor_Control();
                    return 0;
            }

            // Clamp angles (0–180)
            if (servo_x_angle < -100) servo_x_angle = -100;
            if (servo_x_angle > 100) servo_x_angle = 100;
            if (servo_y_angle < -90) servo_y_angle = -90;
            if (servo_y_angle > 90) servo_y_angle = 90;

            // Update servos
            ServoMotor_Control(2, angle_to_pulse(servo_x_angle)); // GPIO18
            ServoMotor_Control(3, angle_to_pulse(servo_y_angle)); // GPIO19

            printf("Servo X: %d°, Servo Y: %d°\r", servo_x_angle, servo_y_angle);
            fflush(stdout);
        }

        usleep(20000); // 20 ms refresh
    }

    return 0;
}