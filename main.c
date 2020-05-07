#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h> //debug
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <sensors/proximity.h>
#include "leds.h"

#include <audio_processing.h>
#include <ir_processing.h>
#include <pi_regulator.h>
#include <fft.h>
#include <arm_math.h>

//global declaration for proximity.h
messagebus_t bus;
MUTEX_DECL(bus_lock); // @suppress("Field cannot be resolved")
CONDVAR_DECL(bus_condvar);

//global var, for first move in intersection

uint8_t first_stop;
uint8_t program_started = 0;

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


int main(void)
{

	//SystemClock_Config();
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    //start the msgbus
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    /*--------------------------stars the thread----------------------------------*/

    pi_regulator_start();
    proximity_start();
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);


    systime_t time;

    /* Infinite loop. */
    while (1) {

    	time = chVTGetSystemTime();
    	chThdSleepUntilWindowed(time, time + MS2ST(100));
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

