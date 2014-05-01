/*
 ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
 2011 Giovanni Di Sirio.

 This file is part of ChibiOS/RT.

 ChibiOS/RT is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 ChibiOS/RT is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the R2D IMU module.
 */

/*
 * Board identifier.
 */
#define BOARD_R2P_PS_MODULE
#define BOARD_NAME              "R2P Power supply module"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            8000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F10X_MD

/*
 * IO pins assignments.
 */
#define LED1_GPIO				GPIOB
#define LED1					1
#define LED2_GPIO				GPIOC
#define LED2					6
#define LED3_GPIO				GPIOC
#define LED3					7
#define LED4_GPIO				GPIOA
#define LED4					8

#define FWD_GPIO				GPIOC
#define FWD						12
#define BWD_GPIO				GPIOD
#define BWD						2
#define BSAFE_GPIO				GPIOC
#define BSAFE					11

#define SERIAL_DRIVER           SD1
#define SPI_DRIVER              SPID2

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything floating input except:

 * PA8  - Normal input (QEI1_A)
 * PA9  - Normal input (QEI1_B)
 * PA11 - Normal input     (CAN1 RX).
 * PA12 - Alternate output (CAN1 TX).
 */
#define VAL_GPIOACRL            0x88888888      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x888B4844      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup.
 * Everything floating input except:
 *
 * PB1  - Push Pull output (LED1)
 * PB6  - Normal input (QEI4_A).
 * PB7  - Normal input (QEI4_B).
 * PB12 - Push Pull output (SPI2 CS).
 * PB13 - Alternate output (SPI2 SCK).
 * PB14 - Normal input     (SPI2 MISO).
 * PB15 - Alternate output (SPI2 MOSI).

 */
#define VAL_GPIOBCRL            0x44888838      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0xB4B38888      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input with pull-up except:
 *
 * PC6  - Push Pull output (LED2)
 * PC7  - Push Pull output (LED3)
 * PC8  - Normal input (ENC_A).
 * PC9  - Normal input (ENC_B).
 * PC11 - Input with pull-up (BSAFE).
 * PC12 - Push Pull output (FWD)
 */
#define VAL_GPIOCCRL            0x33888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x88838844      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFEFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 *
 * PD2  - Push Pull output (BWD)
 */
#define VAL_GPIODCRL            0x88888388      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFB

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
