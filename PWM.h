/* 
 * File:   PWM.h
 * Author: joseg
 *
 * Created on August 2, 2022, 9:10 PM
 */

#ifndef PWM_H
#define	PWM_H


#include <xc.h>
#include <stdint.h>
#define VALOR_PR2 15

//Definimos funciones
void pwm_init(uint8_t channel);
void pwm_duty_cycle(uint16_t duty_cycle, uint8_t channel);
#endif	/* OSCILADOR_H */

