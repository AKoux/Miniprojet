#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include "leds.h"
#include <audio_processing.h>
#include <pid_regulator.h>
#include <ir_processing.h>

//simple PIDregulator implementation
int16_t pid_regulator(float difference){

	int16_t error = 0;
	static int16_t last_error = 0;
	static int16_t derivative = 0;
	static int16_t integral = 0;
	int16_t speed = 0;

	error = difference;
	//disables the regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	derivative = error - last_error;
	integral = integral + error;
	last_error = error;

	speed = KP * error + KI * integral + KD * derivative;
    return (int16_t)speed;
}

static THD_WORKING_AREA(waPidRegulator, 256);
static THD_FUNCTION(PidRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed_correction = 0;


    while(1){
        if(!get_position() && program_started){
        	time = chVTGetSystemTime();

        	first_stop= FS_TO_BE_DONE;

        	clear_leds();

        	//computes a correction factor to let the robot stay in the middle of the corridor
        	speed_correction = pid_regulator(get_side());

        	if(speed_correction > MAX_SPPED_CORR){
        		speed_correction = MAX_SPPED_CORR;
        	    }
        	if(speed_correction < -MAX_SPPED_CORR){
        		speed_correction = -MAX_SPPED_CORR;
        	    }

        	right_motor_set_speed(SPEED_INI  + speed_correction);
        	left_motor_set_speed(SPEED_INI - speed_correction);
        	//100Hz
        	chThdSleepUntilWindowed(time, time + MS2ST(10));

        }
    }

}

void pid_regulator_start(void){
	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO, PidRegulator, NULL);
}
