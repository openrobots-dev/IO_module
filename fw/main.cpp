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

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "R2PMODX"
#endif

#define THROTTLE_CONTROL_FREQUENCY 20


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
/* Robot parameters.                                                         */
/*===========================================================================*/
#define _TICKS 180.0f
#define _PI 3.14159265359f
#define _L 1.70f
#define _B 0.96f // for differential drive kinematics#define R2T(r) ((r / (2 * _PI)) * _TICKS)#define T2R(t) ((t / _TICKS) * (2 * _PI))
#define T2ML(t) (t * 0.0077115867)
#define T2MR(t) (t * 0.0076923077)

#define R2ML(r) (T2ML(R2T(r)))
#define R2MR(r) (T2MR(R2T(r)))

/*===========================================================================*/
/* Application threads.                                                      */
/*===========================================================================*/

/*
 * Pose delta (differential drive kinematics) publisher node.
 */

msg_t delta_pose_differential_node(void * arg) {
	r2p::Node node("pose_d");
	r2p::Subscriber<r2p::Encoder2Msg, 5> wheels_sub;
	r2p::Subscriber<r2p::EncoderMsg, 5> steer_sub;
	r2p::Publisher<r2p::tPose3Msg> delta_pose_pub;
	r2p::tPose3Msg * delta_pose_msgp;
	r2p::EncoderMsg * steer_msgp;
	r2p::Encoder2Msg * wheels_msgp;
	float steer_position = 0.0;

	(void) arg;
	chRegSetThreadName("delta_pose_differential_node");

	node.subscribe(steer_sub, "steer_encoder");
	node.subscribe(wheels_sub, "wheel_encoders");
	node.advertise(delta_pose_pub, "delta_pose_diff");

	chThdSleepMilliseconds(100);

	while ((true)) {
		node.spin(r2p::Time::ms(500));

		while (steer_sub.fetch(steer_msgp)) {
			steer_position = steer_msgp->delta;
			steer_sub.release(*steer_msgp);
		}

		while (wheels_sub.fetch(wheels_msgp)) {
			if (delta_pose_pub.alloc(delta_pose_msgp)) {
				delta_pose_msgp->timestamp.sec = chTimeNow() / CH_FREQUENCY; // conversion to s
				delta_pose_msgp->timestamp.nsec = (chTimeNow() % CH_FREQUENCY)
						* 1000000; // conversion to ns

				float diff = (R2ML(wheels_msgp->right_delta) / THROTTLE_CONTROL_FREQUENCY
						- R2MR(wheels_msgp->left_delta) / THROTTLE_CONTROL_FREQUENCY);

				if (diff == 0.0f) {
					delta_pose_msgp->x = R2ML(wheels_msgp->right_delta) / THROTTLE_CONTROL_FREQUENCY;
					delta_pose_msgp->y = 0;
					delta_pose_msgp->w = 0;
					delta_pose_pub.publish(*delta_pose_msgp);
				} else {
					float radius = _B / 2.0f
							* (R2ML(wheels_msgp->right_delta) / THROTTLE_CONTROL_FREQUENCY
									+ R2MR(wheels_msgp->left_delta) / THROTTLE_CONTROL_FREQUENCY) / diff;

					float alpha = diff / _B;

					delta_pose_msgp->x = radius * sin(alpha);
					delta_pose_msgp->y = radius * cos(alpha);
					delta_pose_msgp->w = alpha;
					delta_pose_pub.publish(*delta_pose_msgp);
				}
			}

			wheels_sub.release(*wheels_msgp);
		}
	}

	return CH_SUCCESS;
}

/*
 * Pose delta (ackermann kinematics) publisher node.
 */

msg_t delta_pose_ackermann_node(void * arg) {
	r2p::Node node("pose_h");
	r2p::Subscriber<r2p::Encoder2Msg, 5> wheels_sub;
	r2p::Subscriber<r2p::EncoderMsg, 5> steer_sub;
	r2p::Publisher<r2p::tPose3Msg> delta_pose_pub;
	r2p::tPose3Msg * delta_pose_msgp;
	r2p::EncoderMsg * steer_msgp;
	r2p::Encoder2Msg * wheels_msgp;
	float steer_position = 0.0f;

	(void) arg;
	chRegSetThreadName("delta_pose_ackermann_node");

	node.subscribe(steer_sub, "steer_encoder");
	node.subscribe(wheels_sub, "wheel_encoders");
	node.advertise(delta_pose_pub, "delta_pose_ack");

	chThdSleepMilliseconds(100);

	while ((true)) {
		node.spin(r2p::Time::ms(500));

		while (steer_sub.fetch(steer_msgp)) {
			steer_position = steer_msgp->delta;
			steer_sub.release(*steer_msgp);
		}

		while (wheels_sub.fetch(wheels_msgp)) {
			if (delta_pose_pub.alloc(delta_pose_msgp)) {
				delta_pose_msgp->timestamp.sec = chTimeNow() / CH_FREQUENCY; // conversion to s
				delta_pose_msgp->timestamp.nsec = (chTimeNow() % CH_FREQUENCY)
						* 1000000; // conversion to ns

				float velocity = (R2ML(wheels_msgp->left_delta)/ THROTTLE_CONTROL_FREQUENCY
						+ R2MR(wheels_msgp->right_delta)/ THROTTLE_CONTROL_FREQUENCY) / 2.0f;
				float orientation = velocity * tanh(steer_position) / _L;

				delta_pose_msgp->x = velocity * cos(orientation);
				delta_pose_msgp->y = velocity * sin(orientation);
				delta_pose_msgp->w = orientation;
				delta_pose_pub.publish(*delta_pose_msgp);
			}

			wheels_sub.release(*wheels_msgp);
		}
	}

	return CH_SUCCESS;
}

/*
 * Velocity publisher node.
 */

msg_t velocity_node(void * arg) {
	r2p::Node node("velocity");
	r2p::Subscriber<r2p::Encoder2Msg, 5> wheels_sub;
	r2p::Subscriber<r2p::EncoderMsg, 5> steer_sub;
	r2p::Publisher<r2p::tVelocity3Msg> velocity_pub;
	r2p::tVelocity3Msg * velocity_msgp;
	r2p::EncoderMsg * steer_msgp;
	r2p::Encoder2Msg * wheels_msgp;
	float steer_position = 0.0f;

	(void) arg;
	chRegSetThreadName("velocity_node");

	node.subscribe(steer_sub, "steer_encoder");
	node.subscribe(wheels_sub, "wheel_encoders");
	node.advertise(velocity_pub, "velocity");

	chThdSleepMilliseconds(100);

	while ((true)) {
		node.spin(r2p::Time::ms(500));

		while (steer_sub.fetch(steer_msgp)) {
			steer_position = steer_msgp->delta;
			steer_sub.release(*steer_msgp);
		}

		while (wheels_sub.fetch(wheels_msgp)) {
			if (velocity_pub.alloc(velocity_msgp)) {
				velocity_msgp->timestamp.sec = chTimeNow() / CH_FREQUENCY; // conversion to s
				velocity_msgp->timestamp.nsec = (chTimeNow() % CH_FREQUENCY)
						* 1000000; // conversion to ns

				float v = (R2ML(wheels_msgp->left_delta)
						+ R2MR(wheels_msgp->right_delta)) / 2.0f;
				velocity_msgp->x = v;
				velocity_msgp->y = 0.0f;
				velocity_msgp->w = v * tanh(steer_position) / _L;
				velocity_pub.publish(*velocity_msgp);
			}

			wheels_sub.release(*wheels_msgp);
		}
	}

	return CH_SUCCESS;
}

/*
 * Throttle control node.
 */
QEIConfig qei1cfg = { QEI_MODE_QUADRATURE, QEI_BOTH_EDGES, QEI_DIRINV_TRUE, };
QEIConfig qei4cfg = { QEI_MODE_QUADRATURE, QEI_BOTH_EDGES, };

const SPIConfig spi2cfg = { NULL, /* HW dependent part.*/GPIOB, 12, SPI_CR1_BR_2
		| SPI_CR1_BR_1 | SPI_CR1_BR_0 };

PID throttle_pid;
r2p::Time last_throttle_setpoint(0);

void throttle_write(SPIDriver *spip, uint8_t data) {
	uint8_t txbuf[2];

	txbuf[0] = 0x11;
	txbuf[1] = data;

	spiSelect(spip);
	halPolledDelay(US2RTT(100));
	spiSend(spip, 2, txbuf);
	halPolledDelay(US2RTT(100));
	spiUnselect(spip);
}

bool throttle_callback(const r2p::Velocity3Msg &msg) {

	if (msg.x > 0.0) {
		palClearPad(BWD_GPIO, BWD); palSetPad(FWD_GPIO, FWD);
		throttle_pid.set(msg.x);
	} else if ((msg.x < 0.0) && (palReadPad(BSAFE_GPIO, BSAFE) == PAL_HIGH)) {
		palClearPad(FWD_GPIO, FWD); palSetPad(BWD_GPIO, BWD);
		throttle_pid.set(-msg.x);
	} else {
		throttle_pid.reset();
		palClearPad(FWD_GPIO, FWD); palClearPad(BWD_GPIO, BWD);
	}

	last_throttle_setpoint = r2p::Time::now();

	/* Turn off RED led. */
	palSetPad(LED4_GPIO, LED4);

	/* Toggle GREEN led. */
	palTogglePad(LED2_GPIO, LED2);

	return true;
}

msg_t throttle_control_node(void * arg) {
	r2p::Node node("velocity");
	r2p::Publisher<r2p::Encoder2Msg> wheels_pub;
	r2p::Subscriber<r2p::Velocity3Msg, 2> vel_sub(throttle_callback);
	r2p::Encoder2Msg *encoder_msgp;
	qeidelta_t left_ticks;
	qeidelta_t right_ticks;
	int16_t throttle_cmd;
	float speed;
	systime_t time;

	(void) arg;
	chRegSetThreadName("throttle_control_node");

	/* Start SPI driver. */
	spiStart(&SPI_DRIVER, &spi2cfg);
	chThdSleepMilliseconds(100);

	qeiStart(&QEID1, &qei1cfg);
	qeiStart(&QEID4, &qei4cfg);

	qeiEnable (&QEID1);
	qeiEnable (&QEID4);

	/* Throttle PID configuration. */
	throttle_pid.config(5.0, 5.0, 0.0, (1.0 / THROTTLE_CONTROL_FREQUENCY), 0.0,
			255.0);

	/* Subscribe and publish topics. */
	node.subscribe(vel_sub, "vel_cmd");
	node.advertise(wheels_pub, "wheel_encoders");

	for (;;) {
		time = chTimeNow();

		/* Invoke callbacks. */
		node.spin(r2p::Time::IMMEDIATE);

		/* Update wheel encoders readings. */
		left_ticks = qeiUpdate(&QEID1) * THROTTLE_CONTROL_FREQUENCY;
		right_ticks = qeiUpdate(&QEID4) * THROTTLE_CONTROL_FREQUENCY;
		speed = (T2ML(left_ticks) + T2MR(right_ticks)) / 2.0f;

		/* Check if we are receiving setpoints. */
		if ((r2p::Time::now() - last_throttle_setpoint) < r2p::Time::ms(500)) {
			/* Recent setpoint, update the controller. */
			throttle_cmd = throttle_pid.update(speed);

			if (throttle_cmd > 0)
				throttle_cmd += 4;

			/* Apply the control action. */
			throttle_write(&SPI_DRIVER, throttle_cmd);
		} else {
			/* Setpoint too old, disable the motor. */
			palClearPad(FWD_GPIO, FWD); palClearPad(BWD_GPIO, BWD);
			throttle_write(&SPI_DRIVER, 0);
			throttle_cmd = 0;

			/* Reset the controller. */
			throttle_pid.reset();

			/* Toggle RED led. */
			palTogglePad(LED4_GPIO, LED4);
		}

		/* Publish encoders readings. */
		if (wheels_pub.alloc(encoder_msgp)) {
			encoder_msgp->left_delta = T2R(left_ticks);
			encoder_msgp->right_delta = T2R(right_ticks);
			wheels_pub.publish(*encoder_msgp);

			/* Toggle ORANGE led. */
			palTogglePad(LED3_GPIO, LED3);
		}

		time += MS2ST(1000 / THROTTLE_CONTROL_FREQUENCY);
		chThdSleepUntil(time);
	}

	return CH_SUCCESS;
}

static msg_t throttle_test_thread(void *arg) {
	uint8_t cnt = 0;

	(void) arg;
	chRegSetThreadName("throttle_test");

	spiStart(&SPI_DRIVER, &spi2cfg);

	while (TRUE) {
		throttle_write(&SPI_DRIVER, cnt);
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

	palClearPad(LED1_GPIO, LED1);palClearPad(LED2_GPIO, LED2);palClearPad(LED3_GPIO, LED3);palClearPad(LED4_GPIO, LED4);
	chThdSleepMilliseconds(500);palSetPad(LED1_GPIO, LED1);palSetPad(LED2_GPIO, LED2);palSetPad(LED3_GPIO, LED3);palSetPad(LED4_GPIO, LED4);

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info),
			r2p::Thread::LOWEST);

	rtcantra.initialize(rtcan_config);

	r2p::Middleware::instance.start();

	r2p::ledsub_conf ledsub_conf = { "led" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO,
			r2p::ledsub_node, &ledsub_conf);

	chThdSleepMilliseconds(100);
	bool diocane = false;
	diocane = r2p::Thread::create_heap(NULL, THD_WA_SIZE(4096), NORMALPRIO + 2,
			throttle_control_node, NULL);
	chThdSleepMilliseconds(100);
	diocane = r2p::Thread::create_heap(NULL, THD_WA_SIZE(8192), NORMALPRIO + 2,
			velocity_node, NULL);
	diocane = r2p::Thread::create_heap(NULL, THD_WA_SIZE(8192), NORMALPRIO + 2,
			delta_pose_differential_node, NULL);
	diocane = r2p::Thread::create_heap(NULL, THD_WA_SIZE(8192), NORMALPRIO + 2,
			delta_pose_ackermann_node, NULL);

	for (;;) {
		if (diocane < 23)
			r2p::Thread::sleep(r2p::Time::ms(500));
	}
	return CH_SUCCESS;
}
}
