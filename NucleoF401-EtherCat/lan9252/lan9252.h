#ifndef __LAN9252_H
#define __LAN9252_H

#include "stm32f4xx_hal.h"
#include <lan9252drv.h>

void init9252(spiCTX* ctx);
void app(spiCTX* ctx);
void sendSensorOne(spiCTX* ctx);

#endif
