#include "lan9252.h"

void init9252(spiCTX* ctx) {
	char buffer[64] = {0};
	ULONG TempLong;

	SPIWriteRegisterDirect (ctx, RESET_CTL, DIGITAL_RST);

	unsigned short i = 0;
	do {
		TempLong.Long = SPIReadRegisterDirect (ctx, RESET_CTL, 4);
		sprintf(buffer, "SPI# (RESET_CTL) 0x%08x \r\n", TempLong.Long);
		HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
		i++;
	} while (((TempLong.Byte[0] & 0x01) != 0x00) && (i != Tout));

	i = 0;
	do {
		TempLong.Long = SPIReadRegisterDirect (ctx, BYTE_TEST, 4);
		sprintf(buffer, "SPI# (BYTE_TEST) 0x%08x \r\n", TempLong.Long);
		HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
		i++;
	} while ((TempLong.Long != 0x87654321) && (i != Tout));

	if (i == Tout) {
		ctx->deviceInitiated = 0x0;
		return;
	}

	i = 0;
	do {
		TempLong.Long = SPIReadRegisterDirect (ctx, HW_CFG, 4);
		sprintf(buffer, "SPI# (HW_CFG) 0x%08x \r\n", TempLong.Long);
		HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
		i++;
	} while (((TempLong.Byte[3] & READY) == 0) && (i != Tout));

	if (i == Tout) {
		ctx->deviceInitiated = 0x0;
		return;
	}

	uint16_t data = 0x01;
	SPIWriteRegisterIndirect(ctx, data, AL_STATUS, sizeof(data));

	data = 0x0;
	SPIWriteRegisterIndirect(ctx, data, AL_STATUS_CODE, sizeof(data));

	ctx->deviceInitiated = 0x1;
}

void printBufferToUart(spiCTX* ctx) {
	char buffer[128] = {0};

	sprintf(buffer, "Buffers: \r\n Out {LED: 0x%02x} \r\n In  {1: 0x%04x, 2: 0x%04x, 3: 0x%04x, 4: 0x%02x, 5: 0x%02x} \r\n",
			ctx->bOut->Cust.Leds,
			ctx->bIn->Cust.Analog_0,
			ctx->bIn->Cust.Analog_1,
			ctx->bIn->Cust.Bit16_RisingTestRamp,
			ctx->bIn->Cust.Bit8_FallingTestRamp,
			ctx->bIn->Cust.DipSwitches);
	HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
}

int direction = -1;
uint64_t indexer = 0;
uint16_t value = 0;
void sendSensorOne(spiCTX* ctx) {
	if (200 == value && direction == 1) {
		direction = -1;
	} else if (0 == value && direction == -1) {
		direction = 1;
	}

	value = (indexer % 200) * direction;

	ctx->bIn->Cust.Analog_0 = value;
	SPIWriteProcRamFifo(ctx);
}

void app(spiCTX* ctx) {
	char buffer[128] = {0};
	ULONG TempLong;

	uint32_t current_tick = HAL_GetTick();

	if (ctx->deviceInitiated) {
		TempLong.Long = SPIReadRegisterIndirect(ctx, WDOG_STATUS, 1); // read the watchdog status
		if ((TempLong.Byte[0] & 0x01) == 0x01) {
			ctx->device.watchdog = 0;	// set/reset the corresponding flag
		} else {
			ctx->device.watchdog = 1;
		}

		TempLong.Long = SPIReadRegisterIndirect (ctx, AL_STATUS, 1);   // read the EtherCAT State Machine status
		ctx->device.registers.status = TempLong.Byte[0] & 0x0F;                         // to see if we are in operational state
		if (ctx->device.registers.status == ESM_OP) {
			ctx->device.oper = 1;
		} else {
			ctx->device.oper = 0;
		}

		SPIReadProcRamFifo(ctx);

		if (60000 == value && direction == 1) {
			direction = -1;
		} else if (0 == value && direction == -1) {
			direction = 1;
		}

		value += 5000 * direction;

		ctx->bIn->Cust.Analog_0 = value;
		// ctx->bIn->Cust.Analog_0 = 100;
		ctx->bIn->Cust.Analog_1 = 200;
		ctx->bIn->Cust.Bit16_RisingTestRamp = 300;
		ctx->bIn->Cust.Bit8_FallingTestRamp = 10;
		ctx->bIn->Cust.DipSwitches = 20;

		SPIWriteProcRamFifo(ctx);
		printBufferToUart(ctx);

		ctx->deviceTime = SPIReadRegisterIndirect(ctx, ECAT_LOCAL_TIME, 4);
		ctx->device.registers.events = SPIReadRegisterIndirect(ctx, AL_EVENT, 2);

		memset(buffer, 0, 64);
		sprintf(buffer, "WD: %d, OPER: %d (STATUS:0x%08x) TIME: 0x%08x, EVENTS: 0x%08x\r\n",
				ctx->device.watchdog,
				ctx->device.oper,
				ctx->device.registers.status,
				ctx->deviceTime,
				ctx->device.registers.events);
		HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
	} else {
		memset(buffer, 0, 64);
		sprintf(buffer, "Something wrong, no communication with device\r\n");
		HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
	}

	while (HAL_GetTick() <= (current_tick + 500));
}
