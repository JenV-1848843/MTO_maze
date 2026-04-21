/* ****************************************************************** */
/*  FILE : motorcontrol.c  for  PI5 BOARD                             */
/*                                                                    */
/* Hardware PWM available on GPIO12, GPIO13, GPIO18, GPIO19           */
/*                                                                    */
/* Outputs:  (DC-motors)                                              */
/*    DC1 > GPIO13,  DC2 > GPIO18,  DC3 >  GPIO19                     */  
/*  Pin 13, 5 and 6 	to control motor1 with hardware PWM1.         */
/*  Pin 18, 16 and 17 	to control motor2 with hardware PWM2.         */
/*  Pin 19, 22 and 23 	to control motor3 with hardware PWM3.         */
/*    DC4 >  GPIO12   (No direction controls!!)                       */
/*                                                                    */
/* Outputs:  for Servo's  at                                          */
/*    Servo 1 > GPIO13,  Servo 2 > GPIO18,  Servo 3 >  GPIO19         */
/*    Servo 4 >  GPIO12                                               */
/*                                                                    */
/*  Date: 18/03/2026                              Johan Baeten        */
/* ****************************************************************** */


#include <stdlib.h> 
#include <stdio.h>
#include <gpiod.h>   //New lib for PI5 chipset!!
#include <unistd.h>  //Needed for sleep and usleep
#include <time.h>
#include <error.h>
#include <string.h>
#include <fcntl.h>

#define PWMCHIP 0
#define CHIPNAME "/dev/gpiochip0"
#define SERVO1 1
#define SERVO2 2
#define SERVO3 3
#define SERVO4 0
#define DCM1 1
#define DCM2 2
#define DCM3 3
#define DCM4 0
#define DCM1CW 5
#define DCM1CCW 6
#define DCM2CW 16
#define DCM2CCW 17
#define DCM3CW 22
#define DCM3CCW 23

struct gpiod_chip *chip; 
struct gpiod_line_request *request;
int prev_direction[3]={0,0,0};


//Functions to support IO and hardware pwm on pi5
//*****************************************

void set_digital_output(int gpiopin, int value){
    if (value == 1) 
	gpiod_line_request_set_value(request, (unsigned int) gpiopin, GPIOD_LINE_VALUE_ACTIVE);
    else 
	gpiod_line_request_set_value(request, (unsigned int) gpiopin, GPIOD_LINE_VALUE_INACTIVE);
    return;
}

void reset_all_digital_outputs(){
    	set_digital_output(DCM1CW, 0);
	set_digital_output(DCM1CCW, 0);
	set_digital_output(DCM2CW, 0);
	set_digital_output(DCM2CCW, 0);
	set_digital_output(DCM3CW, 0);
	set_digital_output(DCM3CCW, 0);
	prev_direction[0]=0;
	prev_direction[1]=0;
	prev_direction[2]=0;
}

void set_pwm_duty_cycle(int pwm_channel, int duty_cycle) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle", PWMCHIP, pwm_channel);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open duty_cycle");
        return;
    }
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d", duty_cycle);
    write(fd, buffer, strlen(buffer));
    close(fd);
}

void set_pwm_period(int pwm_channel, int period) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d/period", PWMCHIP, pwm_channel);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open period");
        return;
    }
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d", period);
    write(fd, buffer, strlen(buffer));
    close(fd);
}

void enable_pwm(int pwm_channel) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d/enable", PWMCHIP, pwm_channel);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open enable");
        return;
    }
    write(fd, "1", 1);
    close(fd);
}

void disable_pwm(int pwm_channel) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm%d/enable", PWMCHIP, pwm_channel);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open enable");
        return;
    }
    write(fd, "0", 1);
    close(fd);
}

void Turn_off_pwm(){
	disable_pwm(1);
	disable_pwm(2);
	disable_pwm(3);
	disable_pwm(0);
	reset_all_digital_outputs();
}

// Servo motor control using hardware pwm
//***************************************
int Init_ServoMotor_Control(void){
	reset_all_digital_outputs();
	disable_pwm(SERVO1);
	disable_pwm(SERVO2);
	disable_pwm(SERVO3);
	disable_pwm(SERVO4);
	set_pwm_period(SERVO1,20000000);  // 20ms
	set_pwm_period(SERVO2,20000000);  // 20ms
	set_pwm_period(SERVO3,20000000);  // 20ms
        set_pwm_period(SERVO4,20000000);  // 20ms
	enable_pwm(SERVO1);
	enable_pwm(SERVO2);
	enable_pwm(SERVO3);
	enable_pwm(SERVO4);
	return 0;
}

void ServoMotor_Control(int motornr, int pulswidth){
	switch (motornr) {
	case 1:
		set_pwm_duty_cycle(SERVO1,pulswidth*1000);
		break;
	case 2:
		set_pwm_duty_cycle(SERVO2,pulswidth*1000);
		break;
	case 3:
		set_pwm_duty_cycle(SERVO3,pulswidth*1000);
		break;
	case 4:
		set_pwm_duty_cycle(SERVO4,pulswidth*1000);
		break;
	}
} 

// DC/PWM motor control using hardware pwm
//****************************************
void set_motor_pwm_period(int chan,int period){
	disable_pwm(chan);
	set_pwm_period(chan,period);  // 100 us - 10 kHz
	enable_pwm(chan);
}

void Init_motors_for_pwm(){
	set_motor_pwm_period(DCM1,100000);
	set_motor_pwm_period(DCM2,100000);  // 100 us - 10 kHz
	set_motor_pwm_period(DCM3,100000);  // 100 us - 10 kHz
	set_motor_pwm_period(DCM4,100000);
	reset_all_digital_outputs();
}

int Init_GPIO_Control(void){
    struct gpiod_line_settings *settings;
    struct gpiod_line_config *line_cfg;
    struct gpiod_request_config *req_cfg;
    unsigned int line_offsets[] = {5,6,16,17,22,23};
    int num_lines = 6;
    
    // Open de chip
    chip =  gpiod_chip_open(CHIPNAME);
    if (!chip) {
        perror("Open chip failed\n");
        return 1;
    }
     // Configureer de instellingen (Output)
    settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    
    // Koppel de instellingen aan alle GPIO outputs
    line_cfg = gpiod_line_config_new();
    gpiod_line_config_add_line_settings(line_cfg, line_offsets, 
                                   num_lines, settings);    
    // Maak het request aan
    req_cfg = gpiod_request_config_new();
    request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

    Init_motors_for_pwm();
    return 0;
}

void Motor_Control(int motor_nr, int velocity, int direction){
    int CW_pin, CCW_pin;
    
    if (motor_nr != 4){  // No direction control for PWM 4
	if (prev_direction[motor_nr-1] != direction){
	    prev_direction[motor_nr-1] = direction;
	    switch (motor_nr) {
	    case 1:
	    default:
		CW_pin = DCM1CW;  CCW_pin  = DCM1CCW;  
		break;
	    case 2: 
		 CW_pin = DCM2CW;  CCW_pin  = DCM2CCW;  
		 break;
	    case 3:
	  	 CW_pin = DCM3CW;  CCW_pin  = DCM3CCW;  
		 break;
	    }

	    if (direction == 1) 	{
		set_digital_output(CW_pin , 1);
		set_digital_output(CCW_pin , 0);
	    } else if (direction == -1) { 
		set_digital_output(CW_pin , 0);
		set_digital_output(CCW_pin , 1);
	    } else {
		set_digital_output(CW_pin , 0);
		set_digital_output(CCW_pin , 0);		
	    }
	}  
    }

	switch (motor_nr) {
	case 1:
		set_motor_pwm_period(DCM1,100*1000);
		set_pwm_duty_cycle(DCM1,velocity*1000);
		break;
	case 2:
		set_motor_pwm_period(DCM2,100*1000);
		set_pwm_duty_cycle(DCM2,velocity*1000);
		break;
	case 3:
		set_motor_pwm_period(DCM3,100*1000);
		set_pwm_duty_cycle(DCM3,velocity*1000);
		break;
	case 4:
		set_motor_pwm_period(DCM4,100*1000);
		set_pwm_duty_cycle(DCM4,velocity*1000);
		break;
	}
	
}

void Set_Motor(int motor, int signed_pwm){
	int mypwm = signed_pwm;
	int direction = 1;
	if (mypwm < 0 ) {
		mypwm = -mypwm;
		direction = -1;
	} else if (mypwm ==0) {
		direction = 0 ;
	}
	Motor_Control(motor, mypwm, direction);
}

void Exit_Motor_Control(){
    gpiod_line_request_release(request);
    gpiod_chip_close(chip);
    return;   
}
