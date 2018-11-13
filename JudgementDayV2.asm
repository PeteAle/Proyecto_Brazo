;*******************************************************************************
;                                                                              *
;    Filename:	    Proyecto Final                                             *
;    Date:          11/11/18                                                   *
;    File Version:                                                             *
;    Author:        Peter Yau 17914, Daniel Cano 17272                         *
;    Company:       UVG                                                        *
;    Description:                                                              *
;                                                                              *
;*******************************************************************************
; TODO Step #1 Processor Inclusion
;*******************************************************************************

#include "p16f887.inc"

;*******************************************************************************
; TODO Step #2 - Configuration Word Setup
;*******************************************************************************

; CONFIG1
; __config 0xFFD4
 __CONFIG _CONFIG1, _FOSC_INTRC_NOCLKOUT & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _BOR4V_BOR40V & _WRT_OFF

;*******************************************************************************
; TODO Step #3 - Variable Definitions
;*******************************************************************************

GPR_VAR	UDATA
STATUS_TEMP RES 1
W_TEMP	    RES 1
CONT1	    RES 1
CONT2	    RES 1

;*******************************************************************************
; Reset Vector
;*******************************************************************************

RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    SETUP                    ; go to beginning of program

;*******************************************************************************
; TODO Step #4 - Interrupt Service Routines
;*******************************************************************************

ISR_VECT  CODE    0x0004
PUSH:
    BCF	    INTCON,0
    BCF	    INTCON,1
    MOVWF   W_TEMP
    SWAPF   STATUS,W
    MOVWF   STATUS_TEMP
ISR:
    
POP2:
    SWAPF   STATUS_TEMP,W
    MOVWF   STATUS
    SWAPF   W_TEMP,F
    SWAPF   W_TEMP,W
    BSF	    INTCON,0
    BSF	    INTCON,1
    RETFIE			    ; RETORNO DE INTERRUPCIóN
    
;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************

MAIN_PROG CODE                      ; let linker place main program

SETUP
    CALL    CONFIG_IO
    CALL    CONFIG_OSC
    CALL    CONFIG_INT
    CALL    CONFIG_ADC
    ;CALL    CONFIG_PWM1
    ;CALL    CONFIG_PWM2
    CALL    CONFIG_RXTX
    
LOOP
;    BCF	    ADCON0,2
    CALL    DELAY
    BSF	    ADCON0,GO
CHECKADC1:
    BTFSC   ADCON0,GO
    GOTO    CHECKADC1
    BCF	    PIR1,ADIF
    MOVF    ADRESH,W
    MOVWF   PORTD
PWM1_RX:
    BTFSS   PIR1,RCIF
    GOTO    PWM1_TX
    MOVF    RCREG,W
    MOVWF   PORTB
;    BCF	    PIR1,ADIF
PWM1_TX:
;    MOVF    ADRESH,W
    MOVF    PORTD,W
    MOVWF   TXREG
    
    BTFSS   PIR1,TXIF
    GOTO    $-1
;    RRF	    ADRESH,F
;    RRF	    ADRESH,F
;    RRF	    ADRESH,W
;    ANDLW   B'00011111'
;    MOVWF   CCPR1L
    
    GOTO    LOOP
    
;LOOP2
;    BSF	    ADCON0,2
;    CALL    DELAY
;    BSF	    ADCON0,GO
;CHECKADC2:
;    BTFSC   ADCON0,GO
;    GOTO    CHECKADC2
;    MOVF    ADRESH,W
;    MOVWF   PORTB
;    BCF	    PIR1,ADIF
;    RRF	    ADRESH,F
;    RRF	    ADRESH,F
;    RRF	    ADRESH,W
;    ANDLW   B'00011111'
;    MOVWF   CCPR2L
;    GOTO    LOOP
    
;*******************************************************************************
; SUBROUTINES
;*******************************************************************************
    
CONFIG_IO
    BANKSEL PORTA
    CLRF    PORTA
    CLRF    PORTB
    CLRF    PORTC
    CLRF    PORTD
    BANKSEL TRISA
    BSF     TRISA,RA0
    BSF     TRISA,RA1
    BSF     TRISA,RA2
    BSF     TRISA,RA3
    CLRF    TRISB
    CLRF    TRISC
    CLRF    TRISD
    BANKSEL ANSEL
    BSF     ANSEL,0
    BSF	    ANSEL,1
    BSF	    ANSEL,2
    BSF	    ANSEL,3
    CLRF    ANSELH
    RETURN
    
CONFIG_OSC
    BANKSEL OSCCON
    BCF	    OSCCON,IRCF2
    BSF	    OSCCON,IRCF1
    BSF	    OSCCON,IRCF0	; OSCILLATOR 500 KHZ
    RETURN

CONFIG_INT
    BANKSEL INTCON
    BCF	    INTCON,7
    BCF	    INTCON,6
    RETURN

CONFIG_ADC
    BANKSEL ADCON0
    MOVLW   B'01000000'	    ; FOSC/8
    MOVWF   ADCON0
    BANKSEL ADCON1
    BCF	    ADCON1,ADFM	    ; Justificado izquierdo
    BCF	    ADCON1,VCFG1    ; Voltaje de referencia como VREF-
    BCF	    ADCON1,VCFG0    ; Voltaje de referencia como VREF+
    BANKSEL ADCON0
    BSF	    ADCON0,ADON	    ; Iniciar ADC
    RETURN
    
CONFIG_PWM1
    BANKSEL PR2
    MOVLW   .155
    MOVWF   PR2   
    
    BANKSEL CCP1CON
    MOVLW   B'00001100'
    MOVWF   CCP1CON
    
    MOVLW   B'00001111'
    MOVWF   CCPR1L
    BCF	    CCP1CON, DC2B0
    BSF	    CCP1CON, DC2B1	    ; LSB del duty cicle
    
    RETURN
    
CONFIG_PWM2
    BANKSEL PR2
    MOVLW   .155
    MOVWF   PR2
    
    BANKSEL CCP2CON
    BSF	    CCP2CON,3
    BSF	    CCP2CON,2
    BSF	    CCP2CON,1
    BSF	    CCP2CON,0
    
    MOVLW   B'00001111'
    MOVWF   CCPR2L
    BCF	    CCP2CON, DC2B0
    BSF	    CCP2CON, DC2B1	    ; LSB del duty cicle
    
    BANKSEL T2CON
    MOVLW   B'00000111'
    MOVWF   T2CON
    BANKSEL PIR1
    BCF     PIR1,TMR2IF
    
    RETURN
    
CONFIG_RXTX
    BANKSEL RCSTA
    BSF	    RCSTA,SPEN		; Activar los módulos de RX,TX
    BCF	    RCSTA,RX9
    BSF	    RCSTA,CREN
    BANKSEL SPBRG
    MOVLW   .2
    MOVWF   SPBRG
    CLRF    SPBRGH		; Baudrate 9600
    BANKSEL BAUDCTL
    BSF	    BAUDCTL,BRG16	; Baudrate de 8-bits
    BANKSEL TXSTA
    ;BCF	    TXSTA,TX9
    BCF	    TXSTA,SYNC
    BSF	    TXSTA,BRGH
    BSF	    TXSTA,TXEN
    
    RETURN
    
;*******************************************************************************
; DELAY
;******************************************************************************* 
    
DELAY   
	MOVLW .17
	MOVWF CONT2
CONFIG1:	
	MOVLW .100
	MOVWF CONT1
RESTA2:    
	DECFSZ	CONT1, F
	GOTO	RESTA2
	DECFSZ	CONT2, F
	GOTO	CONFIG1
	RETURN
    
 END