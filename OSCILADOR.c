/* 
 * File:   general.c
 * Author: joseg
 *
 * Created on July 22, 2022, 10:55 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include "OSCILADOR.h"
#include <xc.h>

void int_osc_MHz(uint8_t freq){
        OSCCONbits.IRCF = freq & 0b111;  // frecuencia del oscilador
        OSCCONbits.SCS = 1;             // Activacion del reloj
}

