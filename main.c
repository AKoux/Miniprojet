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
//#include <stm32f407xx.h>
//#include <gpio.h>

#include <audio_processing.h>
#include <ir_processing.h>
#include <pi_regulator.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>


/*void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin);
void gpio_set(GPIO_TypeDef *port, unsigned int pin);
void gpio_clear(GPIO_TypeDef *port, unsigned int pin);
void gpio_toggle(GPIO_TypeDef *port, unsigned int pin);*/

//global declaration for proximity.h
messagebus_t bus;
MUTEX_DECL(bus_lock); // @suppress("Field cannot be resolved")
CONDVAR_DECL(bus_condvar);

//comment for only microphone movement
#define AUTO_MOVEMENT


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

	//SystemClock_Config();
    halInit();
    chSysInit();
    mpu_init();

    /*gpio_config_output_opendrain(LED1);
    gpio_config_output_opendrain(LED3);
    gpio_config_output_opendrain(LED5);
    gpio_config_output_opendrain(LED7);*/

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

    /*--------------------------stars the thread----------------------------------*/
#ifdef AUTO_MOVEMENT
    pi_regulator_start();
#endif /*PI*/
    proximity_start();
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);



    /* Infinite loop. */
    while (1) {

    	/*gpio_toggle(LED1);
        gpio_toggle(LED3);
        gpio_toggle(LED5);
        gpio_toggle(LED7);*/

        /*
        measure time to do stuff
              	  	chSysLock();
                    //reset the timer counter
                    GPTD12.tim->CNT = 0;

                  XXXXXXXXXXXXXXXX

                    time_mag = GPTD12.tim->CNT;
                    chSysUnlock();
        chprintf((BaseSequentialStream *) &SD3, time magnitude = %d us\n", time_mag);
        */
        chThdYield();
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
/*
void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin)
{
    // Output type open-drain : OTy = 1
    port->OTYPER |= (1 << pin);

    // Output data low : ODRy = 0
    port->ODR &= ~(1 << pin);

    // Pull-up : PUPDRy = 01
    port->PUPDR = (port->PUPDR & ~(3 << (pin * 2))) | (1 << (pin * 2));

    // Output speed highest : OSPEEDRy = 11
    port->OSPEEDR |= (3 << (pin * 2));

    // Output mode : MODERy = 01
    port->MODER = (port->MODER & ~(3 << (pin * 2))) | (1 << (pin * 2));
}


void gpio_set(GPIO_TypeDef *port, unsigned int pin)
{
    port->BSRR = (1 << pin);
}

void gpio_clear(GPIO_TypeDef *port, unsigned int pin)
{
    port->BSRR = (1 << (pin + 16));
}

void gpio_toggle(GPIO_TypeDef *port, unsigned int pin)
{
    if (port->ODR & (1<<pin)) {
        gpio_clear(port, pin);
    } else {
    	gpio_set(port, pin);
    }
}*/
