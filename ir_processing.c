#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <sensors/proximity.h>
//#include <motors.h>
//#include <audio_processing.h>
//#include <communications.h>
#include <ir_processing.h>
#include <arm_math.h>

#define MARGE	0.5		//Marge en % pour comparer les get_prox IR
#define CHECK	5		//Marge de verification avant assurer le croisement (en compare "CHECK" fois les get_prox)

static uint movement = 0;
static int difference = 0;
static uint IR1=0; static uint IR8=0; static uint IR3=0; static uint IR6=0; static float IR_avant;

//----------------------public functions-----------------------------------------------------------

//returns the movement conditions
uint get_movement(void){
	static uint i=0,j=0,l=0,k=0,m=0,n=0,o=0,p=0;

    IR8 = get_prox(ir8);
    IR1 = get_prox(ir1);
    IR3 = get_prox(ir3);
    chprintf((BaseSequentialStream *) &SD3, "IRD%d\n", IR3);
    chprintf((BaseSequentialStream *) &SD3, "\n");
    IR6 = get_prox(ir6);
    chprintf((BaseSequentialStream *) &SD3, "IRG%d\n", IR6);
    chprintf((BaseSequentialStream *) &SD3, "\n");
    IR_avant = (IR1+IR8)/2.; //moyenne des deux capteurs avants
    chprintf((BaseSequentialStream *) &SD3, "IRA%f\n", IR_avant);
    chprintf((BaseSequentialStream *) &SD3, "\n");

/*
    chprintf((BaseSequentialStream *) &SD3, "IRA%f\n", IR_avant);
    chprintf((BaseSequentialStream *) &SD3, "\n");
    chprintf((BaseSequentialStream *) &SD3, "IRD%d\n", IR3);
    chprintf((BaseSequentialStream *) &SD3, "\n");
    chprintf((BaseSequentialStream *) &SD3, "IRG%d\n", IR6);
    chprintf((BaseSequentialStream *) &SD3, "\n");

*/

    if(IR_avant<IR3*(1.-MARGE) && IR_avant<IR6*(1.-MARGE)){
    	i++;
    	j=0; k=0; l=0; m=0; n=0; o=0; p=0;
    	if(i>=CHECK){
    		movement = hallway;
    		//chprintf((BaseSequentialStream *) &SD3, "avancer\n");
    	}
    }
    else if (IR_avant<IR3*(1.-MARGE) && (IR6*(1.+MARGE)>IR_avant && IR_avant>IR6*(1.-MARGE))){
    	j++;
    	i=0; k=0; l=0; m=0; n=0; o=0; p=0;
    	if(j>=CHECK){
    		movement = f_l_turn;
    		//chprintf((BaseSequentialStream *) &SD3, "f_l_turn\n");
    	}
    }
    else if (IR3*(1.+MARGE)>IR_avant && (IR_avant>IR3*(1.-MARGE) && IR_avant<IR6*(1.-MARGE))){
    	k++;
    	i=0; j=0; l=0; m=0; n=0; o=0; p=0;
    	if(k>=CHECK){
    		movement = f_r_turn;
    		//chprintf((BaseSequentialStream *) &SD3, "f_r_turn\n");
    	}
    }
    else if(IR_avant*(1.-MARGE)>IR3 && IR_avant*(1.-MARGE)>IR6){
    	l++;
    	i=0; j=0; k=0; m=0; n=0; o=0; p=0;
    	if(l>=CHECK){
    		movement = l_r_turn;
    		//chprintf((BaseSequentialStream *) &SD3, "l_r_turn\n");
    	}
    }
    else if(IR_avant*(1.-MARGE)>IR3 && (IR6*(1.+MARGE)>IR_avant && IR_avant>IR6*(1.-MARGE))){
    	m++;
    	i=0; j=0; k=0; l=0; n=0; o=0; p=0;
    	if(m>=CHECK){
    		movement = r_turn;
    		//chprintf((BaseSequentialStream *) &SD3, "r_turn\n");
    	}
    }
    else if((IR3*(1.+MARGE)>IR_avant && IR_avant>IR3*(1.-MARGE)) && IR_avant*(1.-MARGE)>IR6){
    	n++;
    	i=0; j=0; k=0; l=0; m=0; o=0; p=0;
    	if(n>=CHECK){
    		movement = l_turn;
    		//chprintf((BaseSequentialStream *) &SD3, "l_turn\n");
    	}
    }
     else if(((IR3*(1.+MARGE)>IR_avant && IR_avant>IR3*(1.-MARGE)) && (IR6*(1.+MARGE)>IR_avant && IR_avant>IR6*(1.-MARGE))) && IR_avant>250){
    	o++;
        i=0; j=0; k=0; l=0; n=0; m=0; p=0;
        if(o>=CHECK){
        	movement = dead_end;
        	//chprintf((BaseSequentialStream *) &SD3, "dead_end\n");
        }
    }
    else {
       p++;
       i=0; j=0; k=0; l=0; n=0; m=0; o=0;
       if(p>=CHECK){
       	movement = f_l_r_turn;
       	//chprintf((BaseSequentialStream *) &SD3, "f_l_r_turn\n");
       }
   }
	return movement;
}

//returns the difference between right and left side of the robot (positif si trop a droite)
int get_side(void){
	IR3 = get_prox(ir3);
	IR6 = get_prox(ir6);

	difference = (IR3-IR6);
	return difference;
}
