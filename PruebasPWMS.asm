;*******************************************************************************
;                                                                              *
;    Filename:	    Proyecto Final                                             *
;    Date:          11/11/18                                                   *
;    File Version:                                                             *
;    Author:        Peter Yau 17914, Daniel Cano 17272                         *
;    Company:       UVG                                                        *
;    Description;*******************************************************************************:                                                              *
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
CONVERSION  RES 1
CONTADOR    RES 1
POT3	    RES 1
POT4	    RES 1
	    
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
    BCF	    INTCON,T0IF
    MOVLW   .248
    MOVWF   TMR0
    INCF    CONTADOR
;    RRF	    POT3,F
;    RRF	    POT3,F
    RRF	    POT3,W
    ANDLW   B'00011111'
    SUBWF   CONTADOR,W
    BTFSC   STATUS,C
    BSF	    PORTD,RD1
    BTFSS   STATUS,C
    BCF	    PORTD,RD1
    
;    RRF	    POT4,F
;    RRF	    POT4,F
;    RRF	    POT4,W
;    ANDLW   B'00011111'
;    SUBWF   CONTADOR,W
;    BTFSC   STATUS,C
;    BSF	    PORTD,RD2
;    BTFSS   STATUS,C
;    BCF	    PORTD,RD2
    
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
    CALL    CONFIG_INT
    CALL    CONFIG_ADC
    CALL    CONFIG_TMR0
;    CALL    CONFIG_RXTX
    CLRF    CONTADOR
    CLRF    POT3
    
PWM3
    CALL    SET_AN2
    CALL    DELAY
    BSF	    ADCON0,GO
CHECKADC3:
    BTFSC   ADCON0,GO
    GOTO    CHECKADC3
    MOVF    ADRESH,W
    MOVWF   POT3
    RRF	    POT3,F
    RRF	    POT3,F
    BCF	    PIR1,ADIF
;CHECK_TX3:
;    BTFSS   PIR1,TXIF
;    GOTO    CHECK_TX3
;    MOVF    PORTD
    
PWM4
    CALL    SET_AN3
    CALL    DELAY
    BSF	    ADCON0,GO
CHECKADC4:
    BTFSC   ADCON0,GO
    GOTO    CHECKADC4
    MOVF    ADRESH,W
    MOVWF   POT4
    BCF	    PIR1,ADIF
    
    GOTO    PWM3

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
    BSF	    TRISA,0	; RA0 INPUT
    BSF	    TRISA,1	; RA1 INPUT
    BSF	    TRISA,2	; RA2 INPUT
    BSF	    TRISA,3	; RA3 INPUT
    CLRF    TRISB
    CLRF    TRISC
    CLRF    TRISD
    BANKSEL ANSEL
    BSF	    ANSEL,0	; RA0 ANALÛGICO
    BSF	    ANSEL,1	; RA1 ANALÛGICO
    BSF	    ANSEL,2	; RA2 ANALÛGICO
    BSF	    ANSEL,3	; RA3 ANALÛGICO
    CLRF    ANSELH
    RETURN
    
CONFIG_OSC
    BANKSEL OSCCON
    BCF	    OSCCON,IRCF2
    BSF	    OSCCON,IRCF1
    BSF	    OSCCON,IRCF0	; OSCILADOR A 500 KHZ
    RETURN
    
CONFIG_INT
    BANKSEL INTCON
    BSF	    INTCON,GIE	    ; ENABLE GLOBAL INTERRUPTIONS
    BCF	    INTCON,PEIE	    ; DISABLE PERIPHERAL INTERRUPTIONS
    RETURN
    
CONFIG_ADC
    BANKSEL ADCON0
    BCF	    ADCON0,ADCS1
    BSF	    ADCON0,ADCS0    ; FOSC/8
    BCF	    ADCON0,CHS3
    BCF	    ADCON0,CHS2
    BCF	    ADCON0,CHS1
    BCF	    ADCON0,CHS0	    ; AN0 CHANNEL
    
    BANKSEL ADCON1
    BCF	    ADCON1,ADFM	    ; LEFT JUSTIFIED
    BCF	    ADCON1,VCFG1    ; VDD INTERNO
    BCF	    ADCON1,VCFG0    ; VSS INTERNO
    
    BANKSEL PIR1
    BCF	    PIR1,ADIF
    
    BANKSEL ADCON0
    BSF	    ADCON0,ADON
    
    RETURN
    
CONFIG_RXTX
    BANKSEL BAUDCTL
    BCF	    BAUDCTL,BRG16
    
    BANKSEL TXSTA
    BCF	    TXSTA,TX9
    BCF	    TXSTA,SYNC
    BSF	    TXSTA,BRGH
    BSF	    TXSTA,TXEN
    
    BANKSEL SPBRG
    MOVLW   .12		; BAUDRATE 2400
    MOVWF   SPBRG
    CLRF    SPBRGH
    
    
    BANKSEL RCSTA
    BCF	    RCSTA,RX9
    BSF	    RCSTA,CREN
    BSF	    RCSTA,SPEN
    
    BANKSEL PIR1
    BCF	    PIR1,RCIF
    BCF	    PIR1,TXIF
    
    RETURN    
    
SET_AN0
    BANKSEL ADCON0
    BCF	    ADCON0,CHS3
    BCF	    ADCON0,CHS2
    BCF	    ADCON0,CHS1
    BCF	    ADCON0,CHS0
    RETURN
    
SET_AN1
    BANKSEL ADCON0
    BCF	    ADCON0,CHS3
    BCF	    ADCON0,CHS2
    BCF	    ADCON0,CHS1
    BSF	    ADCON0,CHS0
    RETURN
    
SET_AN2
    BANKSEL ADCON0
    BCF	    ADCON0,CHS3
    BCF	    ADCON0,CHS2
    BSF	    ADCON0,CHS1
    BCF	    ADCON0,CHS0
    RETURN
    
SET_AN3
    BANKSEL ADCON0
    BCF	    ADCON0,CHS3
    BCF	    ADCON0,CHS2
    BSF	    ADCON0,CHS1
    BSF	    ADCON0,CHS0
    RETURN
    
CONFIG_TMR0
    BANKSEL OPTION_REG
    BCF	    OPTION_REG,T0CS	; INTERNAL CYCLE CLOCK
    BCF	    OPTION_REG,T0SE	; INCREMENT LOW-TO-HIGH
    BCF	    OPTION_REG,PSA	; PRESCALES ASIGNADO A TMR0
    
    BCF	    OPTION_REG,PS2
    BSF	    OPTION_REG,PS1
    BSF	    OPTION_REG,PS0	; PRESCALER 

    BANKSEL INTCON
    BSF	    INTCON,T0IE		; ENABLE TMR0 
    
    BANKSEL TMR0
    CLRF    TMR0
    MOVLW   .248
    MOVWF   TMR0
    
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
