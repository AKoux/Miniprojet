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
//#include <process_image.h>
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

	speed = KP * (0.8 * error + 0.2 * last_error);


	/*
	chprintf((BaseSequentialStream *) &SD3, "error_%f\n", error);
	chprintf((BaseSequentialStream *) &SD3, "sum_error_%f\n", last_error);
	chprintf((BaseSequentialStream *) &SD3, "correction_%f\n", speed);
	*/
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

        	first_stop=1; //
        	//chprintf((BaseSequentialStream *) &SD3, "pi_regulaor thread\n");

        	clear_leds();

        	//computes a correction factor to let the robot rotate to be in front of the line
        	speed_correction = pi_regulator(get_side());

        	if(abs(speed_correction) > MAX_SPPED_CORR){
        		speed_correction = MAX_SPPED_CORR;
        	    }
        	right_motor_set_speed(SPEED_INI  + speed_correction);
        	left_motor_set_speed(SPEED_INI - speed_correction);
        	//100Hz
        	chThdSleepUntilWindowed(time, time + MS2ST(10));

        	//chThdYield();
        }
    }

}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
