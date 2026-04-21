/* ****************************************************************** */
/*  FILE : motorcontrol.h                                             */
/*                                                                    */
/*  Header file for motor and servo control on PI5                    */
/*                                                                    */
/*  Date: 31/03/2026                                                  */
/* ****************************************************************** */

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <gpiod.h>

/* ===================== CONSTANTS ===================== */

// PWM channels
#define PWMCHIP 0

// Servo channels
#define SERVO1 1
#define SERVO2 2
#define SERVO3 3
#define SERVO4 0

// DC motor PWM channels
#define DCM1 1
#define DCM2 2
#define DCM3 3
#define DCM4 0

// Direction control pins
#define DCM1CW 5
#define DCM1CCW 6
#define DCM2CW 16
#define DCM2CCW 17
#define DCM3CW 22
#define DCM3CCW 23

/* ===================== FUNCTION PROTOTYPES ===================== */

/* GPIO / Initialization */
int Init_GPIO_Control(void);
void Exit_Motor_Control(void);

/* General PWM control */
void set_pwm_duty_cycle(int pwm_channel, int duty_cycle);
void set_pwm_period(int pwm_channel, int period);
void enable_pwm(int pwm_channel);
void disable_pwm(int pwm_channel);
void Turn_off_pwm(void);

/* Servo control */
int Init_ServoMotor_Control(void);
void ServoMotor_Control(int motornr, int pulswidth);

/* DC motor control */
void Init_motors_for_pwm(void);
void Motor_Control(int motor_nr, int velocity, int direction);
void Set_Motor(int motor, int signed_pwm);

/* Digital outputs */
void set_digital_output(int gpiopin, int value);
void reset_all_digital_outputs(void);

#endif /* MOTORCONTROL_H */