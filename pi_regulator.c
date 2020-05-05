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
	float speed = 0;

	static float sum_error = 0;

	error = difference;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	/*if(fabs(error) < 300){
			sum_error=0 ;
		}*/
	speed = KP * error;
	//speed = KP * error + KI * sum_error;

	/*chprintf((BaseSequentialStream *) &SD3, "error_%f\n", error);
	chprintf((BaseSequentialStream *) &SD3, "sum_error_%f\n", sum_error);
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

	uint move; uint first_stop=1;

    while(1){
        time = chVTGetSystemTime();

       //100Hz
       // chThdSleepUntilWindowed(time, time + MS2ST(10));
        move=get_movement();
        chprintf((BaseSequentialStream *) &SD3, "pi_regulaor thread\n");

        if(!move){
        	first_stop=1;
        	clear_leds();

        	//computes a correction factor to let the robot rotate to be in front of the line
        	speed_correction = pi_regulator(get_side());

        	/*//if the line is nearly in front of the camera, don't rotate
        	if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        	  }
        	  */
        	if(abs(speed_correction) > 200){
        		speed_correction = 200;
        	    }
        	right_motor_set_speed(SPEED_INI  + ROTATION_COEFF * speed_correction);
        	left_motor_set_speed(SPEED_INI - ROTATION_COEFF * speed_correction);

			chprintf((BaseSequentialStream *) &SD3, "move actif%d\n", 5);
        }
        else if(move && first_stop){
        	chprintf((BaseSequentialStream *) &SD3, "move non actif%d\n", 5);
			first_stop=0;
        	//left_motor_set_pos(-600);
			right_motor_set_speed(+SPEED_INI);
			left_motor_set_speed(+SPEED_INI);
			chThdSleepMilliseconds(700);
			/*while (left_motor_get_pos()<0){
					;
			}
			//chThdSleepMilliseconds(500);*/
        }
        else{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
        }
        chThdSleepUntilWindowed(time, time + MS2ST(10));

        chThdYield();
       //chprintf((BaseSequentialStream *) &SD3, "Position_%d\n", get_movement());

    }

}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
