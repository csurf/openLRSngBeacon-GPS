#ifndef _RFM_H
#define _RFM_H
#include "hardware.h"

// helper macro for European PMR and US FRS channels
#define EU_PMR_CH(x) (445993750L + 12500L * (x)) // valid for ch 1 - 8
#define US_FRS_CH(x) (462537500L + 25000L * (x)) // valid for ch 1 - 7

#define RF22B_PWRSTATE_POWERDOWN    0x00
#define RF22B_PWRSTATE_READY        0x01
#define RF22B_PACKET_SENT_INTERRUPT 0x04
#define RF22B_PWRSTATE_RX           0x05
#define RF22B_PWRSTATE_TX           0x09
#define RF22B_Rx_packet_received_interrupt   0x02

#define NOP() __asm__ __volatile__("nop")


uint8_t ItStatus1, ItStatus2;
uint8_t spiReadRegister(uint8_t address);
void spiWriteRegister(uint8_t address, uint8_t data);
void spiWriteBit(uint8_t b);
void spiSendCommand(uint8_t command);
void spiSendAddress(uint8_t i);
uint8_t spiReadData(void);
void spiWriteData(uint8_t i);

void spiWriteBit(uint8_t b)
{
	if (b) {
		SCK_off;
		NOP();
		SDI_on;
		NOP();
		SCK_on;
		NOP();
	} else {
		SCK_off;
		NOP();
		SDI_off;
		NOP();
		SCK_on;
		NOP();
	}
}

uint8_t spiReadBit(void)
{
	uint8_t r = 0;
	SCK_on;
	NOP();

	if (SDO_1) {
		r = 1;
	}

	SCK_off;
	NOP();
	return r;
}

void spiSendCommand(uint8_t command)
{
	nSEL_on;
	SCK_off;
	nSEL_off;

	for (uint8_t n = 0; n < 8 ; n++) {
		spiWriteBit(command & 0x80);
		command = command << 1;
	}

	SCK_off;
}

void spiSendAddress(uint8_t i)
{
	spiSendCommand(i & 0x7f);
}

void spiWriteData(uint8_t i)
{
	for (uint8_t n = 0; n < 8; n++) {
		spiWriteBit(i & 0x80);
		i = i << 1;
	}

	SCK_off;
}

uint8_t spiReadData(void)
{
	uint8_t Result = 0;
	SCK_off;

	for (uint8_t i = 0; i < 8; i++) {   //read fifo data byte
		Result = (Result << 1) + spiReadBit();
	}

	return(Result);
}

uint8_t spiReadRegister(uint8_t address)
{
	uint8_t result;
	spiSendAddress(address);
	result = spiReadData();
	nSEL_on;
	return(result);
}

void spiWriteRegister(uint8_t address, uint8_t data)
{
	address |= 0x80; //
	spiSendCommand(address);
	spiWriteData(data);
	nSEL_on;
}


// ***** RFM SETUP/CONFIG ROUTINES *****
void rfmSetCarrierFrequency(uint32_t f)
{
	uint16_t fb, fc, hbsel;
	if (f < 480000000) {
		hbsel = 0;
		fb = f / 10000000 - 24;
		fc = (f - (fb + 24) * 10000000) * 4 / 625;
	} else {
		hbsel = 1;
		fb = f / 20000000 - 24;
		fc = (f - (fb + 24) * 20000000) * 2 / 625;
	}
	spiWriteRegister(0x75, 0x40 + (hbsel?0x20:0) + (fb & 0x1f));
	spiWriteRegister(0x76, (fc >> 8));
	spiWriteRegister(0x77, (fc & 0xff));
}

void rfm_init(void)
{
	ItStatus1 = spiReadRegister(0x03);   // read status, clear interrupt
	ItStatus2 = spiReadRegister(0x04);
	spiWriteRegister(0x06, 0x00);    // no wakeup up, lbd,
	spiWriteRegister(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
	spiWriteRegister(0x09, 0x7f);  // (default) c = 12.5p
	spiWriteRegister(0x0a, 0x05);
	spiWriteRegister(0x0b, 0x12);    // gpio0 TX State
	spiWriteRegister(0x0c, 0x15);    // gpio1 RX State
	spiWriteRegister(0x0d, 0xfd);    // gpio 2 micro-controller clk output
	spiWriteRegister(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

	spiWriteRegister(0x70, 0x2C);    // disable manchest
	spiWriteRegister(0x30, 0x00);    //disable packet handling
	spiWriteRegister(0x79, 0);    // start channel
	spiWriteRegister(0x7a, 0x05);   // 50khz step size (10khz x value) // no hopping

	spiWriteRegister(0x71, 0x12);   // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
	spiWriteRegister(0x72, 0x10);   // fd (frequency deviation) 2*625Hz == 1.25kHz

	spiWriteRegister(0x73, 0x00);
	spiWriteRegister(0x74, 0x00);    // no offset

	rfmSetCarrierFrequency(BEACON_FREQUENCY);
}

void rfm_deinit(void)
{
	spiWriteRegister(0x07, RF22B_PWRSTATE_POWERDOWN);
}

void rfm_tx(void)
{
	spiWriteRegister(0x6d, BEACON_POWER_LEVEL);
	delay(10);
	spiWriteRegister(0x07, RF22B_PWRSTATE_TX);    // to tx mode
	delay(10);
}

void rfm_rx(void)
{
  ItStatus1 = spiReadRegister(0x03);
  ItStatus2 = spiReadRegister(0x04);
  spiWriteRegister(0x07, RF22B_PWRSTATE_READY);
  delay(10);
  spiWriteRegister(0x07, RF22B_PWRSTATE_RX);   // to rx mode
  ItStatus1 = spiReadRegister(0x03);   //read the Interrupt Status1 register
  ItStatus2 = spiReadRegister(0x04);
  NOP();
}

uint8_t rfmGetRSSI(void)
{
	return spiReadRegister(0x26);
}

#endif