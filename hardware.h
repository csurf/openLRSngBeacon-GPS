#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#if (BOARD_TYPE == 3)

#define USE_ICP1 // use ICP1 for PPM input for less jitter

#define PPM_IN 8 // ICP1

#define Red_LED A3
#define Green_LED 13

#define Red_LED_ON  PORTC |= _BV(3);
#define Red_LED_OFF  PORTC &= ~_BV(3);    // Was originally #define Green_LED_OFF  PORTB |= _BV(5);   E.g turns it ON not OFF

#define Green_LED_ON  PORTB |= _BV(5);
#define Green_LED_OFF  PORTB &= ~_BV(5);

//## RFM22B Pinouts for Public Edition (Rx v2)
#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on PORTC |= (1<<2) //A2
#define  SCK_off PORTC &= 0xFB //A2

#define  SDI_on PORTC |= (1<<1) //A1
#define  SDI_off PORTC &= 0xFD //A1

#define  SDO_1 (PINC & 0x01) == 0x01 //A0
#define  SDO_0 (PINC & 0x01) == 0x00 //A0

#define SDO_pin A0
#define SDI_pin A1
#define SCLK_pin A2
#define IRQ_pin 2
#define nSel_pin 4

#define IRQ_interrupt 0

#endif

#if (BOARD_TYPE == 5) // openLRSngRX / DTF -4/6ch


#define USE_ICP1 // use ICP1 for PPM input for less jitter
#define PPM_IN 8 // ICP1

#define Red_LED 6
#define Green_LED 5

#define Red_LED_ON    PORTD |=  _BV(6);
#define Red_LED_OFF   PORTD &= ~_BV(6);
#define Green_LED_ON  PORTD |=  _BV(5);
#define Green_LED_OFF PORTD &= ~_BV(5);

//## RFM22B Pinouts for Public Edition (M2)
#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on  PORTB |= _BV(5)  //B5
#define  SCK_off PORTB &= ~_BV(5) //B5

#define  SDI_on  PORTB |= _BV(3)  //B3
#define  SDI_off PORTB &= ~_BV(3) //B3

#define  SDO_1 (PINB & _BV(4)) == _BV(4) //B4
#define  SDO_0 (PINB & _BV(4)) == 0x00  //B4

#define SDO_pin 12
#define SDI_pin 11
#define SCLK_pin 13
#define IRQ_pin 2
#define nSel_pin 4
#endif


#endif