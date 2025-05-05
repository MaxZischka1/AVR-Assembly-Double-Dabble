#include "avr/io.h"
#include  "Timer1prescalers.h"   //local Timer defines
#include  "ADCprescalers.h"     //local ADC defines

.global TIMER1_OVF_vect     ;sufficient for handling this interrupt
.global ADC_vect            ;sufficient for handling this interrupt

.equ BAUD_PRESCALER, 103    ;precomputed for 16MHz OSC & 9600 BAUD
.equ ASYNCHRONOUS, 0<<UMSEL01 | 0<<UMSEL00
.equ SYNCHRONOUS,  0<<UMSEL01 | 1<<UMSEL00
.equ PARITY_NONE,  0<<UPM01   | 0<<UPM00
.equ PARITY_EVEN,  1<<UPM01   | 0<<UPM00
.equ PARITY_ODD,   1<<UPM01   | 1<<UPM00
.equ CHAR_SIZE8,   0<<UCSZ02  | 1<<UCSZ01 | 1<<UCSZ00 ;8-bit
.equ STOP_BITS1,   0<<USBS0
.equ STOP_BITS2,   1<<USBS0
.equ cTHREE,0x03         ;support for DD algorithm
.equ cTHREEZERO,0x30     ;support for DD algorithm
;avr-as syntax for SYMBOLIC Names of GP Registers
util    = 16       ;assigns a symbolic name to a GP Register

offset  = 23       ;

.text               ;position for placement in Program Flash
.org  0x0000        ;position for start of Interrupt Vector Table (IVT)
  rjmp  main        ;
.org  _VECTORS_SIZE ;position beyond the IVT. See iom328p.h
data:
.word   65535       ;ADC Example. Maximum result from an ADC Conversion
converted:
.word   0x5535      ;hardcoded DD Conversion of data(above) for now
.word   0x0006      ;
.org  0x0200        ;provide the origin for the start of assembly code
.equ plHI,  0x00  ;optional preload (0xFDDD yields 120ovf/s=60Hz)
.equ plLO,  0x00  ;optional preload for TCNT1L (0xFD70=50Hz, 0xFCCC=40Hz)


.global setup
setup:
  rcall   IO_setup
  rcall   TIMER1Setup    ;Timer 1 Mode 0 with preload 
  rcall   ADCSetup       ;
  rcall   Init_UART                 //configure the UART for transmission
  

nextChar:
  lpm     r24,Z+                    //load r24 register with the next character
  cpi     r24,0                     //are we at the end of the string?
  breq    bye                       //if so, we're done
  rcall   TXCHAR                    //transmit the contents of r24
  rcall   delay1ms                  //pause...
  rjmp    nextChar                  //keep going...
bye:
  ret                               //end of setup

// Precondition:  Character data (1 byte) to be transmitted is in R24
// Postcondition: Contents of R24 are transmitted via the UART 
TXCHAR:                               
  lds   util,UCSR0A         ;determine if the transmit buffer is empty
  sbrs  util,UDRE0          ;if the flag is set, exit the loop
  rjmp  .-8                 ;loop back to the previous statement 
  sts   UDR0, r24           ;transmit data in r24
  ret


delay1ms:
    ldi  r18, 21
    ldi  r19, 199
L1: dec  r19
    brne L1
    dec  r18
    brne L1
    ret

IO_setup:
  

Init_UART:
 ldi   util,hi8(BAUD_PRESCALER)       ;set the baud rate
  sts   UBRR0H,util                    ;"
  ldi   util,lo8(BAUD_PRESCALER)       ;"
  sts   UBRR0L,util                    ;"

  ldi   util,(1<<RXEN0) | (1<<TXEN0)   ;enable receive and transmit
  sts   UCSR0B,util                    ;"
  
  ldi   util, ASYNCHRONOUS | CHAR_SIZE8 | PARITY_NONE | STOP_BITS1
  sts   UCSR0C,util                    ;set Comm. Mode  Default: 8N1 
  ret

.global loop
loop:
  rjmp loop                           //Avoid returning to the C driver

TIMER1_OVF_vect:          ;destination for TCNT1 overflow interrupt 
  ldi util,plHI           ;load TCNT1 (Timer1's 2-byte counter)
  sts TCNT1H,util         ;
  ldi util,plLO           ;
  sts TCNT1L,util         ;at 60Hz there appears to be some flicker
  reti

display:
  ldi   offset,'0'          ;BCD values need an ASCII '0' (48) offset
  mov   r24,r19            ;prepare the most significant BCD digit
  swap  r24                 ;exchange the high and low nibbles
  andi  r24,0x0F            ;mask off the high nibble 
  add   r24,offset          ;upgrade to ASCII character value
  rcall TXCHAR              ;echo (transmit) it
  mov   r24,r19           ;prepare the BCD 1000s digit 
  andi  r24,0x0F            ;mask off the high nibble
  add   r24,offset          ;upgrade to ASCII character value
  rcall TXCHAR              ;echo (transmit) it
  mov   r24,r18           ;prepare the BCD 100s digit 
  swap  r24                 ;exchange the high and low nibbles
  andi  r24,0x0F            ;mask off the high nibble
  add   r24,offset          ;upgrade to ASCII character value
  rcall TXCHAR              ;echo (transmit) it
  mov   r24,r18           ;prepare the BCD 10s digit 
  andi  r24,0x0F            ;mask off the high nibble
  add   r24,offset          ;upgrade to ASCII character value
  rcall TXCHAR              ;echo (transmit) it
  ldi   r24, '\r'         ;prepare the BCD 1s (units) digit 
  rcall TXCHAR              ;echo (transmit) it
  ldi   r24, '\n'         ;prepare the BCD 1s (units) digit 
  rcall TXCHAR              ;echo (transmit) it
  ret

ADC_vect:                 ;ISR handler for ADC Complete 
  rcall DoubleDabble      ;call algorithm
  rcall display           ;display on monitor
  reti
  
TIMER1Setup:
  clr util                ;prepare for Normal Mode
  sts TCCR1A, util        ;set Normal Mode, now configure the prescaler...
  ldi util, T1ps64        ;T1:2^24/2^8/2^16  (prescale)>1 ovf/s > 0.5Hz
  sts TCCR1B, util        ;Timer1 clock = system clock / prescale
  //ldi util,plHI           ;load TCNT1 (Timer1's 2-byte counter)
  sts TCNT1H,util         ;
  //ldi util,plLO           ;
  sts TCNT1L,util         ;at 60Hz there appears to be some flicker
  ldi util,1<<TOIE1       ;enable Timer/Counter1 Overflow Interrupt
  sts TIMSK1,util         ;enable Timer1 Overflow interrupts
  ret

ADCSetup:
  ser   util                ;r16 <- 0xFF 
  sts   DIDR0, util         ;disable digital pins on PORTC when using ADC
  ldi   util, ADCAVCC       ;AVCC as voltage reference (0.1uF on AREF) 
  ori   util, ADC5          ;choose (arbitrarily) analog pin A5
  ori   util, (1<<ADLAR)    ; enable left-adjust result
  sts   ADMUX,util          ;do it


  ldi   util, 1<<ADEN       ;enable the ADC utility
  ori   util, 1<<ADSC       ;start a first dummy conversion
  ori   util, ADCps128      ;recommended prescaler to get to 125kHz
  sts   ADCSRA, util        ;do it
dummy:
  lds   util,ADCSRA         ;wait for it to complete
  sbrs  util,ADIF           ;is the first 25-cycle conversion complete?
  rjmp  dummy               ;keep waiting....

  lds   util, ADCSRA        ;prepar to clear the ADIF flag
  ori   util, 1<<ADIF       ;ADIF flag is cleared by writing a 1 to it
  ori   util, 1<<ADATE      ;enable external trigger to start a conversion
  sts   ADCSRA,util         ;

  clr   util                ;Free Running Mode (not in this instance)
  ldi   util, ADCtsT1OVF    ;select Timer1 Overflow as Trigger Source
  sts   ADCSRB,util         ;do it

  lds   util,ADCSRA        ;need to enable ADC Complete Interrupt...
  ori   util, 1<<ADIE       ;set the interrupt Enable bit...
  sts   ADCSRA,util         ;do it
  ret

DoubleDabble:
  lds r16, ADCL   ;load bin6 and 7
  lds r17, ADCH   ;load all bins
  clr r18         ;clr register     
  clr r19              
  ldi r20, 10     ;loop counter; 10 shifts     
first:
  mov r21, r19  ;Doubledabble thousands
  swap r21      ;move high nibble to low nibble
  andi r21, 0x0F  ;bit mask
  cpi r21, 5  ;check if BCD val >5
  brlo second ;if lower do not add three
  subi r21, -3  ;add three
  swap r21      ;move back to high nibble
  andi r21, 0xF0    ;make sure nothing interfers with lower
  andi r19, 0x0F
  or   r19, r21     ;add together
second:
  mov r21, r19  ;Doubledabble hundreds
  andi r21, 0x0F  ;bit mask
  cpi r21, 5      ;check if BCD val >5
  brlo third      ;if lower do not add three
  subi r19, -3    ;add three
third:
  mov r21, r18  ;Doubledabble tens
  swap r21      ;move high nibble to low nibble
  andi r21, 0x0F  ;bit mask
  cpi r21, 5      ;check if BCD val >5
  brlo fourth   ;if lower do not add three
  subi r21, -3  ;add three
  swap r21      ;move back to high nibble
  andi r21, 0xF0  ;make sure nothing interfers with lower
  andi r18, 0x0F  
  or   r18, r21   ;add together
fourth:
  mov r21, r18  ;Doubledabble ones
  andi r21, 0x0F  ;bit mask
  cpi r21, 5      ;check if BCD val >5
  brlo shift      ;if lower do not add three
  subi r18, -3    ;add three
shift:
  lsl r16   ;shift entire value onces
  rol r17
  rol r18
  rol r19
  dec r20   ;decrement after 1 shift
  brne first  
  ret