/*
 * File:   lab1.c
 * Author: ECNG2005
 *
 * Created on October 18, 2023, 5:17 PM
 */

//Zarius Ali
//816032702

#include <xc.h>
#define _XTAL_FREQ 4000000

void main(void) {
    TRISDbits.TRISD2=0;
    LATDbits.LATD2=1;
   
    while(1==1)
    {
        LATDbits.LATD2=0;
        __delay_ms(1000);
         LATDbits.LATD2=1;
        __delay_ms(1000);
    }
       
      return;  
    }
