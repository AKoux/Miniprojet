#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include "leds.h"
#include <audio/microphone.h>
#include <audio_processing.h>
#include <ir_processing.h>
#include <fft.h>
#include <arm_math.h>


//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		60	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	66	//1031.25 [Hz] (= 66*15.625, 15.625 is the resolution)	C6
#define FREQ_LEFT		72	//1125 [Hz]												D6
#define FREQ_RIGHT		78	//1218 [HZ]												D#6/Eb6
#define FREQ_BACKWARD	84	//1312.5 [Hz]											E6
#define MAX_FREQ		90	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)

//MOVE STEPS

#define FRONT_SHORT 	340 //before audio choice
#define FRONT_LONG 		450 //after audio choice
#define ROT 			325 //90 deg rotation, experimentation: + 1 steps


#define	DISABLE_DIR	0
#define	ENABLE_DIR	1


//static values to translate get_position(), used in 3 different functions.
static uint8_t forward	= 0;
static uint8_t left		= 0;
static uint8_t right 	= 0;
static uint8_t backward = 0;
static uint8_t deadend 	= 0;

enum DISPLACEMENT{forward_move, left_turn, right_turn, backward_turn, forward_initial, dead_end_turn}; //all the possible displacement with audio
/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*	data is micleft_output, the magnitudes of frequencies
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	direction_enable(get_position());
	direction_led();
	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	//go forward if want to and can
	if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){

		if(forward && program_started){
			audio_displacement(forward_move);
		}
		else{
			program_started = ON; //first frequency has been produced to lunch the program.
		}
	}
	//turn left if want to and can
	else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
		if(left){
			audio_displacement(left_turn);
		}
	}
	//turn right if want to and can
	else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
		if(right){
			audio_displacement(right_turn);
		}
	}
	//go backward if want to and can
	else if(max_norm_index >= FREQ_BACKWARD_L && max_norm_index <= FREQ_BACKWARD_H){
		if(deadend){
			audio_displacement(dead_end_turn);
		}
		else if(backward && !program_started){
			audio_displacement(backward_turn);
		}
	}
	else{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		first_stop = FS_DONE;
	}
	right = DISABLE_DIR; left = DISABLE_DIR; forward = DISABLE_DIR; backward = DISABLE_DIR; deadend = DISABLE_DIR;
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
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*/

	static uint16_t nb_samples = 0;
	uint8_t move = get_position();

	if(move || !program_started){ //process audio only if in cross-road

		if(first_stop && !(move==dead_end)){
			audio_displacement(forward_initial);
			first_stop = FS_DONE;
		}

		//loop to fill the buffers
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

			nb_samples = 0;

			sound_remote(micLeft_output);
		}
	}
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

//enable and disable the directions corresponding to the position.
void direction_enable(uint direction){
	switch(direction)
	{
		case hallway:
			forward = ENABLE_DIR;
			backward = ENABLE_DIR;
			break;
		case l_turn:
			left = ENABLE_DIR;
			backward = ENABLE_DIR;
			break;
		case r_turn:
			right = ENABLE_DIR;
			backward = ENABLE_DIR;
			break;
		case l_r_turn:
			right = ENABLE_DIR;
			left = ENABLE_DIR;
			backward = ENABLE_DIR;
			break;
		case f_l_turn:
			left = ENABLE_DIR;
			forward = ENABLE_DIR;
			backward = ENABLE_DIR;
			break;
		case f_r_turn:
			right = ENABLE_DIR;
			forward = ENABLE_DIR;
			backward = ENABLE_DIR;
			break;
		case f_l_r_turn:
			left = ENABLE_DIR;
			right = ENABLE_DIR;
			forward = ENABLE_DIR;
			backward = ENABLE_DIR;
			break;
		case dead_end:
			backward = ENABLE_DIR;
			deadend = ENABLE_DIR;
			break;
	}
}

void direction_led(void){

	clear_leds();

	if(forward==DISABLE_DIR){
    	set_led(LED1, ON);
	}
	if(left==DISABLE_DIR){
    	set_led(LED7, ON);
	}
	if(right==DISABLE_DIR){
    	set_led(LED3, ON);
	}
	if(backward==DISABLE_DIR){
    	set_led(LED5, ON);
	}
}

void audio_displacement(uint displacement){

	switch(displacement)
	{
		case forward_move: //goes forward
			move_forward(-FRONT_LONG);
			break;

		case left_turn:
			rotation(left_turn, ROT);
			move_forward(-FRONT_LONG);
			break;

		case right_turn:
			rotation(right_turn, ROT);
			move_forward(-FRONT_LONG);
			break;

		case backward_turn:
			rotation(left_turn, 2*ROT);
			move_forward(-FRONT_LONG);
			break;

	    case forward_initial: //just before crossroad (goes in the middle)
			move_forward(-FRONT_SHORT);
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			break;

	    case dead_end_turn:
			rotation(left_turn, 2*ROT);
	    	break;
	}
}

void rotation(uint direction, uint rot_step){
	if(direction == left_turn){
		left_motor_set_pos(rot_step);
		right_motor_set_speed(+SPEED_INI/2);
		left_motor_set_speed(-SPEED_INI/2);
		while (left_motor_get_pos()>0){;}
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}
	else if(direction == right_turn){
		right_motor_set_pos(rot_step);
	    right_motor_set_speed(-SPEED_INI/2);
	    left_motor_set_speed(+SPEED_INI/2);
	    while (right_motor_get_pos()>0){;}
	    right_motor_set_speed(0);
	    left_motor_set_speed(0);
	}
}

void move_forward(uint forward_step){
	left_motor_set_pos(forward_step);
	right_motor_set_speed(+SPEED_INI);
	left_motor_set_speed(+SPEED_INI);
	while (left_motor_get_pos()<0){;}
}
