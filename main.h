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
#define ERROR_THRESHOLD			40.0f
#define ROTATION_THRESHOLD		10
#define KP						0.04f
#define KI 						0.01f	//must not be zero
#define SPEED_INI				400
#define MAX_SPPED_CORR 			(SPEED_INI/8)

#define ON						1
#define OFF						0

enum MOVEMENT{hallway, l_turn, r_turn, l_r_turn, f_l_turn, f_r_turn, f_l_r_turn, dead_end}; //(l=left, r=right, f=forward)
/** Robot wide IPC bus. */

extern messagebus_t bus;

extern uint first_stop;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
