#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include "leds.h"
#include <audio_processing.h>
#include <pi_regulator.h>
#include <ir_processing.h>

//simple PI regulator implementation
int16_t pi_regulator(float difference){

	float error = 0;
	static float last_error = 0;
	float speed = 0;

	last_error = error;

	error = difference;

	//disables the regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	speed = KP * (ERROR_COEF * error + LAST_ERROR_COEF * last_error);

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed_correction = 0;



    while(1){
        if(!get_position()){
        	time = chVTGetSystemTime();

        	first_stop= FS_TO_BE_DONE;

        	clear_leds();

        	//computes a correction factor to let the robot stay in the middle of the corridor
        	speed_correction = pi_regulator(get_side());

        	if(speed_correction > MAX_SPPED_CORR){
        		speed_correction = MAX_SPPED_CORR;
        	    }
        	if(speed_correction < -MAX_SPPED_CORR){
        		speed_correction = -MAX_SPPED_CORR;
        	    }

        	right_motor_set_speed(SPEED_INI  + speed_correction);
        	left_motor_set_speed(SPEED_INI - speed_correction);
        	//100Hz
        	chThdSleepUntilWindowed(time, time + MS2ST(1));


        }
    }

}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
