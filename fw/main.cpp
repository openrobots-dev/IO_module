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

#include "ch.h"
#include "hal.h"

#include "rtcan.h"
#include "config.h"

#include <r2p/common.hpp>
#include <r2p/Middleware.hpp>
#include <r2p/Node.hpp>
#include <r2p/Topic.hpp>
#include <r2p/Publisher.hpp>
#include <r2p/Subscriber.hpp>
#include <r2p/Mutex.hpp>
#include <r2p/NamingTraits.hpp>
#include <r2p/Bootloader.hpp>
#include "r2p/transport/RTCANTransport.hpp"

#include "r2p/msg/motor.hpp"
#include "r2p/node/led.hpp"
#include "r2p/node/motor.hpp"
#include "r2p/msg/std_msgs.hpp"
#include "r2p/node/pid.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "R2PMODX"
#endif

extern "C" {
void *__dso_handle;
void __cxa_pure_virtual() {
	chSysHalt();
}
void _exit(int) {
	chSysHalt();
	for (;;) {
	}
}
int _kill(int, int) {
	chSysHalt();
	return -1;
}
int _getpid() {
	return 1;
}
} // extern "C"

static WORKING_AREA(wa_info, 2048);

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME,
		"BOOT_"R2P_MODULE_NAME);

// RTCAN transport
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

/*===========================================================================*/
/* Application threads.                                                      */
/*===========================================================================*/

/*
 * Digital potentiometer subscriber node.
 */
//const SPIConfig spi2cfg = { NULL, /* HW dependent part.*/GPIOB, 12, SPI_CR1_BR_2 |  SPI_CR1_BR_1 | SPI_CR1_CPOL
//		| SPI_CR1_CPHA };

const SPIConfig spi2cfg = { NULL, /* HW dependent part.*/GPIOB, 12, SPI_CR1_BR_2 |  SPI_CR1_BR_1 |  SPI_CR1_BR_0};


void write(SPIDriver *spip, uint8_t data) {
	uint8_t txbuf[2];

	txbuf[0] = 0xEE;
	txbuf[1] = data;

	spiUnselect(spip);
	halPolledDelay(US2RTT(100));
	spiSend(spip, 2, txbuf);
	halPolledDelay(US2RTT(100));
	spiSelect(spip);
}

bool throttle_callback(const r2p::Velocity3Msg &msg) {

	write(&SPI_DRIVER, 0xFF);
	palTogglePad(LED2_GPIO, LED2);
	palSetPad(LED4_GPIO, LED4);

	return true;
}

msg_t throttle_node(void * arg) {
	r2p::Node node("throttle");
	r2p::Subscriber<r2p::Velocity3Msg, 2> vel_sub(throttle_callback);

	(void) arg;

	chRegSetThreadName("throttle_node");

	/* Start SPI driver. */
	spiStart(&SPI_DRIVER, &spi2cfg);
	chThdSleepMilliseconds(100);

	node.subscribe(vel_sub, "vel_cmd");

	for (;;) {
		if (!node.spin(r2p::Time::ms(500))) {
			/* Stop!! */
			write(&SPI_DRIVER, 0xFF);
			palTogglePad(LED4_GPIO, LED4);
		}
	}

	return CH_SUCCESS;
}

static msg_t mcp41xxx_test_thread(void *arg) {
	uint8_t cnt = 0;

	(void) arg;
	chRegSetThreadName("mcp41xxx");

	spiStart(&SPI_DRIVER, &spi2cfg);

	while (TRUE) {
		write(&SPI_DRIVER, cnt);
		cnt += 16;
		palTogglePad(LED2_GPIO, LED2);
		chThdSleepMilliseconds(500);
	}

	return 0;
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	palClearPad(LED1_GPIO, LED1);
	palClearPad(LED2_GPIO, LED2);
	palClearPad(LED3_GPIO, LED3);
	palClearPad(LED4_GPIO, LED4);
	chThdSleepMilliseconds(500);
	palSetPad(LED1_GPIO, LED1);
	palSetPad(LED2_GPIO, LED2);
	palSetPad(LED3_GPIO, LED3);
	palSetPad(LED4_GPIO, LED4);

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info),
			r2p::Thread::LOWEST);

	rtcantra.initialize(rtcan_config);

	r2p::Middleware::instance.start();

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO,
			r2p::ledsub_node, NULL);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 1,
			throttle_node, NULL);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}
	return CH_SUCCESS;
}
}
