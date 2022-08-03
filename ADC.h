/* 
 * File:   ADC.h
 * Author: joseg
 *
 * Created on July 22, 2022, 11:24 AM
 */

#ifndef ADC_H
#define	ADC_H


#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>


void adc_init (uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus);
void adc_start(uint8_t channel);
int adc_read (void);

#endif	/* ADC_H */

