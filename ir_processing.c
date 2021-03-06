#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <sensors/proximity.h>
#include <ir_processing.h>


#define CHECK	5		//Verification margin before confirm the position state (we "CHECK" 5 consecutive times the get_prox())

#define MUR_PROCHE	280 //to modify according to the environment

static uint left_proche; static uint forward_proche; static uint right_proche; //used in two functions

/*--------------------------------functions--------------------------------------*/

//returns the position/crossroad
uint get_position(void){

	static uint8_t position = 0;
	static uint8_t i=0,j=0,l=0,k=0,m=0,n=0,o=0,p=0; //simple counters

    ir_condition();

    if((left_proche && !forward_proche) && right_proche){
    	i++;
    	j=0; k=0; l=0; m=0; n=0; o=0; p=0;
    	if(i>=CHECK){
    		position = hallway;
    	}
    }
    else if ((!left_proche && !forward_proche) && right_proche){
    	j++;
    	i=0; k=0; l=0; m=0; n=0; o=0; p=0;
    	if(j>=CHECK){
    		position = f_l_turn;
    	}
    }
    else if ((left_proche && !forward_proche) && !right_proche){
    	k++;
    	i=0; j=0; l=0; m=0; n=0; o=0; p=0;
    	if(k>=CHECK){
    		position = f_r_turn;
    	}
    }
    else if((!left_proche && forward_proche) && !right_proche){
    	l++;
    	i=0; j=0; k=0; m=0; n=0; o=0; p=0;
    	if(l>=CHECK){
    		position = l_r_turn;
    	}
    }
    else if((left_proche && forward_proche) && !right_proche){
    	m++;
    	i=0; j=0; k=0; l=0; n=0; o=0; p=0;
    	if(m>=CHECK){
    		position = r_turn;
    	}
    }
    else if((!left_proche && forward_proche) && right_proche){
    	n++;
    	i=0; j=0; k=0; l=0; m=0; o=0; p=0;
    	if(n>=CHECK){
    		position = l_turn;
    	}
    }
     else if((left_proche && forward_proche) && right_proche){
    	o++;
        i=0; j=0; k=0; l=0; n=0; m=0; p=0;
        if(o>=CHECK){
        	position = dead_end;
        }
    }
    else {
       p++;
       i=0; j=0; k=0; l=0; n=0; m=0; o=0;
       if(p>=CHECK){
    	   position = f_l_r_turn;
       }
   }
	return position;
}

//returns the difference between right and left side of the robot
//(positive if too far right)
int get_side(void){
	return (get_prox(ir3)-get_prox(ir6));
}

//updates values used in get_position()
void ir_condition(void){
	static uint16_t IR1=0; static uint16_t IR8=0; static uint16_t IR3=0;
	static uint16_t IR6=0; static float IR_avant;

	left_proche 	= OFF;
	right_proche 	= OFF;
	forward_proche 	= OFF;

    IR8 = get_prox(ir8);
    IR1 = get_prox(ir1);
    IR3 = get_prox(ir3); 	 //right
    IR6 = get_prox(ir6); 	 //left
    IR_avant = (IR1+IR8)/2.; //moyenne des deux capteurs avants


	if(IR6>MUR_PROCHE)left_proche = ON;
	if(IR3>MUR_PROCHE)right_proche = ON;
	if(IR_avant>MUR_PROCHE)forward_proche = ON;
}
