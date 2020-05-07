#ifndef MAIN_H
#define MAIN_H



#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the different parts of the lil' project

enum IR{ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};
#define ERROR_THRESHOLD			15.f			//experimental
#define KP						1.f			//experimental
#define SPEED_INI				400				//choice
#define MAX_SPPED_CORR 			(SPEED_INI/16)	//experimental

#define ON						1
#define OFF						0

#define FS_DONE					0
#define FS_TO_BE_DONE			1

enum MOVEMENT{hallway, l_turn, r_turn, l_r_turn, f_l_turn, f_r_turn, f_l_r_turn, dead_end}; //(l=left, r=right, f=forward) all the possible positions/crossroads.

/** Robot wide IPC bus. **/

extern messagebus_t bus;

extern uint8_t first_stop; //for a initial advancement when arriving at a crossroad

extern parameter_namespace_t parameter_root;



#endif
