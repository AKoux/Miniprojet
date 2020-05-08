#ifndef MAIN_H
#define MAIN_H



#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the different parts of the lil' project

enum IR{ir1, ir2, ir3, ir4, ir5, ir6, ir7, ir8};

//in relation with speed of the robot
#define ERROR_THRESHOLD			20.f			//experimental
#define KP						0.7f			//experimental
#define KI						0.0001f			//experimental
#define KD						20.f			//experimental
#define SPEED_INI				400				//choice
#define MAX_SPPED_CORR 			(SPEED_INI/12)	//experimental

#define ON						1
#define OFF						0

//for first_Stop variable
#define FS_DONE					0
#define FS_TO_BE_DONE			1

enum POSITION{hallway, l_turn, r_turn, l_r_turn, f_l_turn, f_r_turn, f_l_r_turn, dead_end}; //(l=left, r=right, f=forward) all the possible positions/crossroads.

/** Robot wide IPC bus. **/

extern messagebus_t bus;

extern uint8_t first_stop; 		//for a preliminary advancement when arriving at a crossroad
extern uint8_t program_started; //wait for first order when placing robot in maze

extern parameter_namespace_t parameter_root;

#endif
