/*
 * pi_regulator.h
 *
 *  Created on: 21 avr. 2020
 *      Author: alexander/re
 */

#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//determined experimentally
#define ERROR_COEF 0.8
#define LAST_ERROR_COEF 0.2

//start the PI regulator thread
void pi_regulator_start(void);

#endif /* PI_REGULATOR_H */
