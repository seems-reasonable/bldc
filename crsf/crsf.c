#include "crsf/crsf.h"

#include "crsf/crc8.h"
#include "mc_interface.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"

#include <string.h>

static volatile bool crsf_is_running = false;

static SerialConfig uart_cfg = {
		416666,
		0,
		USART_CR2_LINEN,
		0
};

void crsf_init(void) {
	crc8_init(0xd5);
}

void crsf_start(void) {
	sdStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	crsf_is_running = true;
}

void crsf_stop(void) {
	if (!crsf_is_running) {
		return;
	}
	sdStop(&HW_UART_DEV);
	crsf_is_running = false;
}

struct __attribute__((packed)) crsf_gps {
    int32_t latitude;       // degree / 10`000`000
    int32_t longitude;      // degree / 10`000`000
    uint16_t groundspeed;   // km/h / 100
    uint16_t heading;       // degree / 100
    uint16_t altitude;      // meter - 1000m offset
    uint8_t satellites;     // # of sats in view
};

static uint16_t convert16(uint16_t v) {
	return ((uint16_t)(v & 0xFF) << 8) | (((uint16_t)(v >> 8)) & 0xFF);
}

static uint32_t convert32(uint32_t v) {
	return ((uint32_t)(v & 0xFF) << 24) |
		((((uint32_t)(v >> 8)) & 0xFF) << 16) |
		((((uint32_t)(v >> 16)) & 0xFF) << 8) |
		(((uint32_t)(v >> 24)) & 0xFF);
}

void crsf_publish_telemetry(void) {
	uint8_t buf[sizeof(struct crsf_gps)+4];
	buf[0] = 0XC8;  // sync byte
	buf[1] = sizeof(struct crsf_gps) + 2;  // type, payload, CRC
	buf[2] = 0x02;  // GPS
	{
		struct crsf_gps *gps = (struct crsf_gps *)&buf[3];
		memset(gps, 0, sizeof(*gps));
		gps->latitude = convert32((uint32_t)(mc_interface_get_rpm() * 65536.0f));
		gps->longitude = convert32((uint32_t)(mc_interface_get_tot_current_directional_filtered() * 65536.0f));
		gps->heading = convert16((uint16_t)(GET_INPUT_VOLTAGE() * 256.0f));
	}
	buf[sizeof(struct crsf_gps) + 3] = crc8_calc(&buf[2], sizeof(struct crsf_gps) + 1);
	sdWrite(&HW_UART_DEV, buf, sizeof(buf));
}
