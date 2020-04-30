#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <sensors/proximity.h>

#include <audio_processing.h>
#include <ir_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>



//declaration globale pour proximity.h askip
messagebus_t bus;
MUTEX_DECL(bus_lock); // @suppress("Field cannot be resolved")
CONDVAR_DECL(bus_condvar);

//uncomment to send the FFTs results from the real microphones
#define SPACIAL_MOVEMENT


static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void)
{
	/*initialisation pour le IR sensor
	int IR1=0; int IR8=0; int IR3=0; int IR6=0; float IR_avant; messagebus_init(&bus, &bus_lock, &bus_condvar);
	int speed = 600;*/


    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    //inits the motors
    motors_init();
    //start the msgbus
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //temp tab used to store values in complex_float format
    //needed bx doFFT_c
    static complex_float temp_tab[FFT_SIZE];
    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];

    //stars the thread----------------------------------
#ifdef SPACIAL_MOVEMENT
    pi_regulator_start();
#endif /*PI*/
    proximity_start();
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);



    /* Infinite loop. */
    while (1) {

       /* //waits until a result must be sent to the computer
        wait_send_to_computer();

        //we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
        SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
		*/


        /*    mesurer time to do stuff
         *       chSysLock();
                    //reset the timer counter
                    GPTD12.tim->CNT = 0;

                   stufffffff

                    time_mag = GPTD12.tim->CNT;
                    chSysUnlock();
        chprintf((BaseSequentialStream *) &SDU1, "time fft = %d us, time magnitude = %d us\n",time_fft, time_mag);*/


        //chprintf((BaseSequentialStream *) &SD3, "Position_%d\n",get_movement());
        chThdYield();


    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
