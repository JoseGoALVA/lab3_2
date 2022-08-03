/* 
 * File:   TMR0.h
 * Author: joseg
 *
 * Created on August 2, 2022, 9:08 PM
 */

#ifndef TMR0_H
#define	TMR0_H


#include <xc.h>
#include <stdint.h>

//Definimos las funciones
void tmr0_init(uint16_t prescaler);
void tmr0_reload(void);
#endif	/* OSCILADOR_H */

