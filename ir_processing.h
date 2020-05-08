/*
 * ir_processing.h
 *
 *  Created on: 21 avr. 2020
 *      Author: alexander/re
 */

#ifndef IR_PROCESSING_H_
#define IR_PROCESSING_H_

//returns the position of the robot (which crossroad)
uint get_position(void);
//returns the difference between right and left side of the robot
int get_side(void);
//updates values used in get_position()
void ir_condition(void);

#endif /* IR_PROCESSING_H_ */
