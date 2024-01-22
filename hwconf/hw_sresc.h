/*
	Copyright 2023 Brian Silverman	brian@seemsreasonable.net

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_SRESC_H_
#define HW_SRESC_H_

#define HW_NAME					"SRESC"

#include "drv8320s.h"

// HW properties
// This hardware actually has DRV8350S, but the registers are the same as
// the DRV8320S (along with the DRV8323S and DRV8353S).
#define HW_HAS_DRV8320S
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
//#define HW_HAS_PHASE_FILTERS

// Docs at https://www.ti.com/lit/ds/symlink/drv8320.pdf and
// https://www.ti.com/lit/ds/symlink/drv8353.pdf
//
// Nominal Q_GD is 34 nC for each FET, which is 68 nC total.
// Nominal Q_G is 169 nC (211 nC) for each FET.
//
// TODO: Lower t_DRIVE to something more reasonable?
// TODO: Pick a reasonable VDS_LVL default.
// TODO: Change OCP_MODE default to LATCH_SHUTDOWN.
//
// VDS_LVL and OCP_MODE from here get overwritten via the motorconf. Note that
// the VDS_LVL value is the OCP_ADJ value divided by 2.
#define DRV8320S_CUSTOM_SETTINGS() do { \
    drv8320s_write_reg(5, ((0 << 10) | (1 << 8) | (1 << 4))); \
    drv8320s_write_reg(3,0x174); \
    drv8320s_write_reg(4,0x374); \
    drv8320s_write_reg(2, (1 << 10)); \
} while (0)

#define LED_GREEN_GPIO			GPIOB
#define LED_GREEN_PIN			5
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				7

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define PHASE_FILTER_GPIO		GPIOC
#define PHASE_FILTER_PIN		9
#define PHASE_FILTER_ON()		palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
#define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)

#define CURRENT_FILTER_ON()		palSetPad(GPIOD, 2)
#define CURRENT_FILTER_OFF()	palClearPad(GPIOD, 2)

/*
 * ADC Vector
 *
 * 0  (1):	IN0		SENS1
 * 1  (2):	IN1		SENS2
 * 2  (3):	IN2		SENS3
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 6  (1):	Vrefint
 * 7  (2):  IN9		TEMP_MOS_3
 * 8  (3):	IN3		TEMP_MOS
 * 9  (1):  IN8		TEMP_MOS_2
 * 10 (2):	IN0		SENS1 (unused)
 * 11 (3):	IN13	AN_IN
 */

#define HW_ADC_CHANNELS			12
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			4

// ADC Indexes
#define ADC_IND_SENS1			0
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			2
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5
#define ADC_IND_VIN_SENS		11
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOS_2		9
#define ADC_IND_TEMP_MOS_3		7
#define ADC_IND_VREFINT			6

// ADC macros and settings

#ifndef V_REG
// Datasheet has this equation: 0.768 * (1 + 33.2/10) = 3.318
// But it measures closer to this, and the datasheet also mentions 0.8 which
// gives 3.346.
// The datasheet has no tolerance or explicitly-stated nominal feedback
// comparison voltage...
#define V_REG					3.34
#endif
#ifndef VIN_R1
#define VIN_R1					55800.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		(0.0005 / 3.0)
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_TEMP(adc_ind)		hw_sresc_get_temp()

#define NTC_RES(adc_val)		((4096.0 * 10000.0) / (4096.0 - (adc_val + 0.5)) - 10000.0)
#define NTC_TEMP_MOSx(adc_ind)			(1.0 / ((logf(NTC_RES(ADC_Value[(adc_ind)]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS1() NTC_TEMP_MOSx(ADC_IND_TEMP_MOS)
#define NTC_TEMP_MOS2() NTC_TEMP_MOSx(ADC_IND_TEMP_MOS_2)
#define NTC_TEMP_MOS3() NTC_TEMP_MOSx(ADC_IND_TEMP_MOS_3)

// Motor temperature sensing: not supported.
#define NTC_TEMP_MOTOR(beta)	(-20)
#define NTC_TEMP_MOTOR_2(beta)	(-20)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif

// UART Peripheral
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// Permanent UART Peripheral (for NRF51)
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		10
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11

// SPI for DRV8350S (same interface as DRV8320S)
//
// drv_8320 configures these.
#define DRV8320S_MOSI_GPIO			GPIOA
#define DRV8320S_MOSI_PIN			7
#define DRV8320S_MISO_GPIO			GPIOA
#define DRV8320S_MISO_PIN			6
#define DRV8320S_SCK_GPIO			GPIOA
#define DRV8320S_SCK_PIN			5
#define DRV8320S_CS_GPIO			GPIOA
#define DRV8320S_CS_PIN				4

#define ENABLE_GATE()				palSetPad(GPIOC, 12)
#define DISABLE_GATE()				palClearPad(GPIOC, 12)
#define IS_DRV_FAULT()			(!palReadPad(GPIOA, 15))

// ICU Peripheral for servo decoding (PPM input, used by app_ppm).
//
// servo_dec configures this pin.
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

#if 0
// SPI pins (hooked up to DRV8350S, which means they can't be used here).
//
// Several other components configure these if they're going to be used.
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6
#endif

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
// The gate driver handles this, so the STM32 drives a minimal amount of deadtime.
#define HW_DEAD_TIME_NSEC		100.0
// TODO: Should we also override MCCONF_FOC_DT_US based on DEAD_TIME (in register 5)?

// Default setting overrides
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			28.0		// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			72.0	// Maximum input voltage
#endif
#ifndef MCCONF_M_DRV8301_OC_ADJ
// TODO: Measure and adjust, accounting for measurement tolerance and
// FET temperature.
#define MCCONF_M_DRV8301_OC_ADJ		10
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_SW
#define MCCONF_FOC_F_SW					40000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		420.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			250.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-200.0	// Input current limit in Amperes (Lower)
#endif

// Setting limits
#define HW_LIM_CURRENT			-400.0, 400.0
#define HW_LIM_CURRENT_IN		-400.0, 400.0
#define HW_LIM_CURRENT_ABS		0.0, 480.0
#define HW_LIM_VIN				24.0, 74.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0

// HW-specific functions
float hw_sresc_get_temp(void);

#endif /* HW_SRESC_H_ */
