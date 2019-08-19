#ifndef __LAN9252DRV_H
#define __LAN9252DRV_H

#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal.h"

#define SPI_TIMEOUT_MAX				0x1000
#define CMD_LENGTH					((uint16_t)0x0004)

#define COMM_SPI_READ				0x03
#define COMM_SPI_WRITE				0x02

#define ESC_WRITE 		   			0x80
#define ESC_READ 		   			0xC0
#define ECAT_CSR_BUSY     			0x80

#define AL_CONTROL              	0x0120      			// AL control
#define AL_STATUS               	0x0130      			// AL status
#define AL_STATUS_CODE          	0x0134      			// AL status code
#define AL_EVENT                	0x0220      			// AL event request
#define AL_EVENT_MASK           	0x0204      			// AL event interrupt mask

#define TOT_BYTE_NUM_OUT			4
#define TOT_BYTE_NUM_IN				8
#define TOT_BYTE_NUM_ROUND_OUT		4
#define TOT_BYTE_NUM_ROUND_IN		8

#define PRAM_ABORT        			0x40000000
#define PRAM_BUSY         			0x80
#define PRAM_AVAIL        			0x01
#define READY             			0x08
#define DUMMY_BYTE					0xFF

#define BYTE_TEST               	0x0064      			// byte order test register
#define HW_CFG                  	0x0074      			// hardware configuration register
#define RESET_CTL               	0x01F8      			// reset register
#define ECAT_CSR_DATA           	0x0300      			// EtherCAT CSR Interface Data Register
#define ECAT_CSR_CMD            	0x0304      			// EtherCAT CSR Interface Command Register
#define ECAT_PRAM_RD_ADDR_LEN   	0x0308      			// EtherCAT Process RAM Read Address and Length Register
#define ECAT_PRAM_RD_CMD        	0x030C      			// EtherCAT Process RAM Read Command Register
#define ECAT_PRAM_WR_ADDR_LEN   	0x0310      			// EtherCAT Process RAM Write Address and Length Register
#define ECAT_PRAM_WR_CMD        	0x0314      			// EtherCAT Process RAM Write Command Register
#define WDOG_STATUS             	0x0440      			// watch dog status
#define ECAT_LOCAL_TIME				0x0910

#define DIGITAL_RST       			0x00000001

#define ESM_INIT                	0x01          			// state machine control
#define ESM_PREOP               	0x02          			// (state request)
#define ESM_BOOT                	0x03          			//
#define ESM_SAFEOP              	0x04          			// safe-operational
#define ESM_OP                  	0x08          			// operational

#define Tout 						1000

typedef union {
    unsigned short  Word;
    unsigned char   Byte[2];
} UWORD;

typedef union {
    unsigned long   Long;
    unsigned short  Word[2];
    unsigned char   Byte[4];
} ULONG;

typedef union {
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct {
		uint8_t     Leds;
	} Cust;
	uint32_t data;
} PROCBUFFER_OUT;

typedef union {
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct {
		uint16_t    Analog_0;
		uint16_t    Analog_1;
		uint16_t    Bit16_RisingTestRamp;
		uint8_t     DipSwitches;
		uint8_t     Bit8_FallingTestRamp;
	} Cust;
	uint64_t data;
} PROCBUFFER_IN;

typedef struct {
	UART_HandleTypeDef* uart;
	SPI_HandleTypeDef* 	spi;
	PROCBUFFER_IN* 		bIn;
	PROCBUFFER_OUT* 	bOut;
	uint8_t				deviceInitiated;
	uint32_t			deviceTime;
	struct {
		uint8_t	oper;
		uint8_t	watchdog;
		struct {
			uint8_t 	status;
			uint16_t	events;
		} registers;
	} device;
} spiCTX;

unsigned long SPIReadRegisterDirect(spiCTX* ctx, unsigned short Address, unsigned char Len);
void SPIWriteRegisterDirect (spiCTX* ctx, unsigned short Address, unsigned long DataOut);
unsigned long SPIReadRegisterIndirect (spiCTX* ctx, unsigned short Address, unsigned char Len);
void SPIWriteRegisterIndirect (spiCTX* ctx, unsigned long DataOut, unsigned short Address, unsigned char Len);
void SPIWriteProcRamFifo(spiCTX* ctx);
void SPIReadProcRamFifo(spiCTX* ctx);

#endif
