#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdint.h>
/*------------------------------------------------------------------------------
 * LIBRERIAS
 ------------------------------------------------------------------------------*/
#include "OSCILADOR.h"
#include "ADC.h"
#include "TMR0.h"
#include "PWM.h"
#include "LCD.h"

/*------------------------------------------------------------------------------
 * BITS DEL DISPLAY
 ------------------------------------------------------------------------------*/

#define _XTAL_FREQ 4000000
#define RS RC6
#define EN RC7
#define D0 RD0
#define D1 RD1
#define D2 RD2
#define D3 RD3
#define D4 RD4
#define D5 RD5
#define D6 RD6
#define D7 RD7

/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
unsigned short VOLTAJE1 = 0;
unsigned short VOLTAJE2 = 0;

uint8_t contador = 0;
uint8_t canal_ADC = 0;
uint8_t VALORPOT1 = 0;
uint8_t VALORPOT2 = 0;
uint8_t entero_POT = 0;
uint8_t DECIMALES1 = 0;
uint8_t entero_POT2 = 0;
uint8_t DECIMALES2 = 0;
uint8_t bandera = 0;
uint8_t ciclo_trabajo = 0;

char s[];

void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, //Función del mapeo
            unsigned short out_min, unsigned short out_max);

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}


void __interrupt() isr (void){
    if(INTCONbits.T0IF){ //Interrupción TMR0?
        contador++;
        if(contador == 100){ //cuenta hasta 5s
            if(bandera == 1){ //primer estado
                bandera = 0;
            }
            else if(bandera == 0){ //segundo estado
                bandera = 1;
            }
            contador = 0;
        }
        tmr0_reload();
    }
    if(PIR1bits.ADIF){ //verificamos canal y leemos con el ADC
        if(canal_ADC == 0){
            VALORPOT1 = adc_read();
        }
        else if(canal_ADC == 1){
            VALORPOT2 = adc_read();
        }
    }
    return;
}
/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    unsigned int a;
    Lcd_Init(); //inicialización
    Lcd_Clear(); //limpiamos LCD
    Lcd_Set_Cursor(1,1); //escojemos en que fila escribir
    Lcd_Write_String("JOSE GONZALEZ ");
    Lcd_Set_Cursor(2,1); //escojemos en que fila escribir
    Lcd_Write_String("CARNE: 20194");
    __delay_ms(2000); //esperamos 2 segundos
    
    
    while(1){
        if (canal_ADC == 0){ //primer canal
            adc_start(canal_ADC);
            canal_ADC = 1;
            VOLTAJE1 = map(VALORPOT1, 0, 255, 0, 500); //mapeamos el voltaje 0 255 minimo 0 500 maximo
            entero_POT = VOLTAJE1/100; //se guarda el entero
            DECIMALES1 = VOLTAJE1-entero_POT*100; //se guerda los decimales como si fueran enteros 
        }
        else if (canal_ADC == 1){//segundo canal
            adc_start(canal_ADC);
            canal_ADC = 0;
            VOLTAJE2 = map(VALORPOT2, 0, 255, 0, 500);  //mapeamos el voltaje 0 255 minimo 0 500 maximo
            entero_POT2 = VOLTAJE2/100; //se guarda el entero
            DECIMALES2 = VOLTAJE2-entero_POT2*100;  //se guerda los decimales como si fueran enteros 
        }
        if(bandera == 1){
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("POTENCIOMETRO 1");
            Lcd_Set_Cursor(2,1);
            sprintf(s, "%d.%dV", entero_POT, DECIMALES1); //guardamos el string que vamos a mostrar
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String(s);
        }
        else if(bandera == 0){
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("POTENCIOMETRO 2");
            Lcd_Set_Cursor(2,1);
            sprintf(s, "%d.%dV", entero_POT2, DECIMALES2); //guardamos el string que vamos a mostrar
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String(s);
        }
        
    }   
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void setup(void){
    TRISD = 0x00;
    TRISC = 0x00;
    ANSELH = 0;
    TRISB = 0;
    PORTB = 0;
    int_osc_MHz(2); // 0 ---> 1MHz, 1 ---> 2MHz, 2 ---> 4MHz, 3 ---> 8MHz, 4 ---> 500kHz, default ---> 4MHz
    adc_init(1,0,0);
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIE1bits.RCIE = 1;
    INTCONbits.T0IE = 1; //Habiliatamos int. TMR0
    tmr0_reload();   //función de TMR0
    tmr0_init(256);  //configuración prescaler 256 TMR0 
}


