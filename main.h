#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the differents parts of the miniproject

enum IR{ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};
#define ERROR_THRESHOLD			100.0f
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			1
#define KP						0.1f
#define KI 						0.01f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define SPEED_INI				600
/** Robot wide IPC bus. */

/*#define LED1     	GPIOD, 5
#define LED3     	GPIOD, 6
#define LED5     	GPIOD, 10
#define LED7     	GPIOD, 11*/

extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
