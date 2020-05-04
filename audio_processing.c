#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <ir_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];


#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		30	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	36	//562.5Hz (= 16*15.625, the resolution)
#define FREQ_LEFT		42	//656.25Hz
#define FREQ_RIGHT		48	//750HZ
#define FREQ_BACKWARD	54	//843.75Hz
#define MAX_FREQ		60	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)

#define	DISABLE_DIR	0
#define	ENABLE_DIR	1

static uint forward = 0;
static uint left = 0;
static uint right = 0;
static uint backward = 0;
/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*	data is micleft_output, the magnitudes of frequencies
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 
	//int16_t speed_ini = 600;

	direction_enable(get_movement());
	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	//go forward if want to and can
	if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){
		if(forward){
			audio_displacement(0);
		}
	}
	//turn left if want to and can
	else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
		if(left){
			audio_displacement(1);
		}
	}
	//turn right if want to and can
	else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
		if(right){
			audio_displacement(2);
		}
	}
	//go backward if want to and can
	else if(max_norm_index >= FREQ_BACKWARD_L && max_norm_index <= FREQ_BACKWARD_H){
		if(backward){
			audio_displacement(3);
		}
	}
	else{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	right = DISABLE; left = DISABLE; forward = DISABLE; backward = DISABLE;
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/



	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	if(get_movement()){ //process audio only if in cross-road
	//loop to fill the buffers

		chprintf((BaseSequentialStream *) &SD3, "process son, move:%d\n", get_movement());

		for(uint16_t i = 0 ; i < num_samples ; i+=4){
			//construct an array of complex numbers. Put 0 to the imaginary part
			micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];


			nb_samples++;

			micLeft_cmplx_input[nb_samples] = 0;

			nb_samples++;

			//stop when buffer is full
			if(nb_samples >= (2 * FFT_SIZE)){
				break;
			}
		}

		if(nb_samples >= (2 * FFT_SIZE)){
			/*	FFT processing
			 *
			 *	This FFT function stores the results in the input buffer given.
			 *	This is an "In Place" function.
			 */

			doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
			/*	Magnitude processing
			 *
			 *	Computes the magnitude of the complex numbers and
			 *	stores them in a buffer of FFT_SIZE because it only contains
			 *	real numbers.
			 *
			 */
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);


			//sends only one FFT result over 10 for 1 mic to not flood the computer
			//sends to UART3
			if(mustSend > 8){
				//signals to send the result to the computer
				chBSemSignal(&sendToComputer_sem);
				mustSend = 0;
			}
			nb_samples = 0;
			mustSend++;

			sound_remote(micLeft_output);
		}
	}

}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else{
		return NULL;
	}
}

void direction_enable(uint direction){
	switch(direction)
	{
					case 0:
						forward = ENABLE;
						backward = ENABLE;
						break;
					case 1:
						left = ENABLE;
						backward = ENABLE;
						break;
					case 2:
						right = ENABLE;
						backward = ENABLE;
						break;
					case 3:
						right = ENABLE;
						left = ENABLE;
						backward = ENABLE;
						break;
					case 4:
						left = ENABLE;
						forward = ENABLE;
						backward = ENABLE;
						break;
					case 5:
						right = ENABLE;
						forward = ENABLE;
						backward = ENABLE;
						break;
					case 6:
						left = ENABLE;
						right = ENABLE;
						forward = ENABLE;
						backward = ENABLE;
						break;
	}
}

void audio_displacement(uint displacement){

	switch(displacement)
	{
		case 0: //goes forward
			left_motor_set_pos(-600);
			right_motor_set_speed(+SPEED_INI);
			left_motor_set_speed(+SPEED_INI);
			while (left_motor_get_pos()<0){
					;
			}
			right_motor_set_speed(0);
			left_motor_set_speed(0);

			break;
		case 1: //turns left
			left_motor_set_pos(324);
			right_motor_set_speed(+SPEED_INI/2);
			left_motor_set_speed(-SPEED_INI/2);
			while (left_motor_get_pos()>0){
					;
			}
			left_motor_set_pos(-600);
			right_motor_set_speed(+SPEED_INI);
			left_motor_set_speed(+SPEED_INI);
			while (left_motor_get_pos()<0){
					;
			}
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			break;
		case 2: //turns right
			right_motor_set_pos(324);
		    right_motor_set_speed(-SPEED_INI/2);
		    left_motor_set_speed(+SPEED_INI/2);
			while (right_motor_get_pos()>0){
					;
			}
			left_motor_set_pos(-600);
			right_motor_set_speed(+SPEED_INI);
			left_motor_set_speed(+SPEED_INI);
			while (left_motor_get_pos()<0){
					;
			}
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			break;
		case 3: //turns backward
			left_motor_set_pos(2*324);
		     right_motor_set_speed(+SPEED_INI/2);
		     left_motor_set_speed(-SPEED_INI/2);
			while (left_motor_get_pos()>0){
					;
			}
			left_motor_set_pos(-600);
			right_motor_set_speed(+SPEED_INI);
			left_motor_set_speed(+SPEED_INI);
			while (left_motor_get_pos()<0){
					;
			}
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			break;
	}
}
