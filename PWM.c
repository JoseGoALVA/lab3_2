#include <xc.h>
#include "PWM.h"
#define _XTAL_FREQ 4000000

void pwm_init(uint8_t channel){
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP1
    PR2 = VALOR_PR2;                  // cambiar a consante
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    switch(channel){
        case 1:
            CCP1CON = 0;                // Apagamos CCP1
            CCP1CONbits.P1M = 0;        // Modo single output
            CCP1CONbits.CCP1M = 0b1100; // PWM
            TRISCbits.TRISC2 = 0;
            break;
        case 2:
            CCP2CON = 0;                // Apagamos CCP1
            CCP2CONbits.CCP2M = 0b1100; // PWM
            TRISCbits.TRISC1 = 0;
            break;
        default:
            break;
             
    }
    
}

void pwm_duty_cycle(uint16_t duty_cycle, uint8_t channel){
    if (duty_cycle < 1024){ //rango para duty cycle
        switch(channel){
            case 1:
                CCPR1L = duty_cycle>>2;
                CCP1CONbits.DC1B = duty_cycle & 0b11;    // ancho de pulso / % ciclo de trabajo
                break;
            case 2:    
                CCPR2L = duty_cycle>>2;
                CCP2CONbits.DC2B0 = duty_cycle & 0b1; // ancho de pulso / % ciclo de trabajo
                CCP2CONbits.DC2B1 = duty_cycle>>1 & 0b1;
                break;
            default:
                break;
        }
    }
}

