;*******************************************************************************
;                                                                              *
;    Filename:	    Mini                                                  *
;    Date:          30/10/18                                                     *
;    File Version:                                                             *
;    Author:        Peter Yau 17914                                            *
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
CONTADOR    RES 1
    
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
    RETFIE			    ; RETORNO DE INTERRUPCIÛN
    
;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************

MAIN_PROG CODE                      ; let linker place main program

SETUP
    CALL    CONFIG_IO
    CALL    CONFIG_OSC
    CALL    CONFIG_ADC
    CALL    CONFIG_PWM
    CALL    CONFIG_RT
LOOP
    BSF	    ADCON0,GO
CHECKADC:
    BTFSC   ADCON0,1
    GOTO    CHECKADC
    MOVF    ADRESH,W
    MOVWF   CONTADOR
    GOTO    LOOP
    
;*******************************************************************************
; SUBROUTINES
;*******************************************************************************
    
CONFIG_IO    
    BCF	    STATUS,RP0
    BCF	    STATUS,RP1	; Acceso Banco 0
    CLRF    PORTA
    CLRF    PORTB
    CLRF    PORTC
    CLRF    PORTD
    BSF	    STATUS,RP0	; Acceso Banco 1
    MOVLW   0FFH
    MOVWF   TRISA	; Port A inputs
    CLRF    TRISB
    BCF	    TRISC,1
    BCF	    TRISC,2
    CLRF    TRISD
    BSF	    STATUS,RP1	; Acceso Banco 3
    MOVLW   B'00101111'	; Port A analogs
    MOVWF   ANSEL
    CLRF    ANSELH
    RETURN
    
CONFIG_OSC
    BANKSEL OSCCON
    BCF	    OSCCON,IRCF2
    BSF	    OSCCON,IRCF1
    BSF	    OSCCON,IRCF0    ; Set oscillator to 500 KHz.
    BSF	    OSCCON,SCS	    ; Use internal oscillator.
    RETURN

CONFIG_ADC
    BANKSEL ADCON0
    MOVLW   B'01000000'
    MOVWF   ADCON0	    ; Fosc/8, Channel AN0 
    BANKSEL ADCON1	  
    BCF	    ADCON1,ADFM	    ; Left justified
    BCF	    ADCON1,VCFG1    ; Vss reference same as internal voltage.
    BCF	    ADCON1,VCFG0    ; Vdd reference same as internal ground.
    BANKSEL ADCON0
    BSF	    ADCON0,ADON	    ; ADC enable
    RETURN

CONFIG_PWM
    BANKSEL PR2
    MOVLW   .155
    MOVWF   PR2
    
    BANKSEL CCP2CON
    BSF	    CCP2CON,3
    BSF	    CCP2CON,2
    BSF	    CCP2CON,1
    BSF	    CCP2CON,0	; PWM Mode
    
    MOVLW   B'00001111'
    MOVWF   CCPR2L
    BCF	    CCP2CON, DC2B0
    BSF	    CCP2CON, DC2B1	    ; LSB del duty cycle
    
    BANKSEL T2CON
    MOVLW   B'00000111'
    MOVWF   T2CON
    BANKSEL PIR1
    BCF     PIR1,TMR2IF
    
    RETURN
    
CONFIG_RT
    
 END