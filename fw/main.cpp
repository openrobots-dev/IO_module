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

#include "r2p/node/led.hpp"
#include "r2p/msg/motor.hpp"

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "IO"
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

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

// RTCAN transport
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

/*===========================================================================*/
/* Steer absolute encoder node.                                              */
/*===========================================================================*/

#define ADC_NUM_CHANNELS 2
#define ADC_BUF_DEPTH 1

#define CH1_MIN		1359
#define CH1_CENTER	3088
#define CH2_MIN		1345
#define CH2_CENTER	3183
#define SMOOTH_WINDOW		0.4f

#define _PI			3.14159265359f


static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH] = { 0 };

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    IN10.
 */

static const ADCConversionGroup adc_grpcfg = {
  TRUE,
  ADC_NUM_CHANNELS,
  NULL,
  NULL,
  0, /* CR1 */
  0, /* CR2 */
  ADC_SMPR1_SMP_AN14(ADC_SAMPLE_239P5), /* SMPR1 */
  ADC_SMPR2_SMP_AN8(ADC_SAMPLE_239P5),
  ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS),
  0, /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN8)   | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN14)
};
/*
 * Encoder thread.
 */
static msg_t steer_absolute_encoder_node(void *arg) {
	r2p::Node node("steerabs");
	r2p::Publisher<r2p::AbsoluteEncoder> encoder_pub;

	(void) arg;
	chRegSetThreadName("steer_absolute_encoder_node");

	node.advertise(encoder_pub, "abs_encoder");

	/*
	 * Activates the ADC1 driver.
	 */
	adcStart(&ADC_DRIVER, NULL);
	adcStartConversion(&ADC_DRIVER, &adc_grpcfg, adc_samples, ADC_BUF_DEPTH);

	while (TRUE) {
		r2p::AbsoluteEncoder * msgp;
		float ch1_position;
		float ch2_position;
		float ch1_weight;
		float ch2_weight;
		float weight_sum;
		systime_t time;

		time = chTimeNow();

		ch1_position = (float)(adc_samples[0] - CH1_CENTER) / (CH1_MIN - CH1_CENTER) * 1.0f; // 1 rad
		ch2_position = -(float)(adc_samples[1] - CH2_CENTER) / (CH2_MIN - CH2_CENTER) * 1.0f; // 1 rad

		ch1_weight = ch1_position > -SMOOTH_WINDOW ? fabs(ch1_position + SMOOTH_WINDOW) : 0.0f;
		ch2_weight = ch2_position < SMOOTH_WINDOW ? fabs(ch2_position - SMOOTH_WINDOW) : 0.0f;
		weight_sum = ch1_weight + ch2_weight;
		ch1_weight /= weight_sum;
		ch2_weight /= weight_sum;

		float steer_position = ch1_position * ch1_weight + ch2_position * ch2_weight;

		if (encoder_pub.alloc(msgp)) {
			msgp->position = -steer_position; // Quadrivio has CW positive angles
			encoder_pub.publish(*msgp);
			palTogglePad(LED2_GPIO, LED2);
		}

		time += MS2ST(10);
		chThdSleepUntil(time);
	}
	return 0;
}

/*===========================================================================*/
/* Speed encoder node.                                                       */
/*===========================================================================*/
#include "stm32_tim.h"

#define T2R(t) (t / 1)

void configure_tim4(void) {

	rccEnableTIM4(FALSE);
    rccResetTIM4();

    STM32_TIM4->CCMR1 = TIM_CCMR1_CC1S_0 | STM32_TIM_CCMR1_IC1F(0x9); // Input on TI1 (PB7), Fsamp = CK_INT/8, Nfilt = 8
	STM32_TIM4->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_TS_0 | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
	STM32_TIM4->ARR = 0xFFFF;
	STM32_TIM4->CR1 = TIM_CR1_CEN;
}

int16_t delta_tim4(void) {
	static uint16_t last_cnt = 0;
	uint16_t cnt;
	int16_t delta;

	chSysLock();
	cnt = STM32_TIM4->CNT;
	delta = cnt - last_cnt;
//	last_cnt = cnt;
	chSysUnlock();

	return delta;
}

msg_t speed_encoder_node(void *arg) {
	r2p::Node node("speed");
	r2p::Publisher<r2p::EncoderMsg> vel_pub;
	systime_t time;
	r2p::EncoderMsg *msgp;

	(void) arg;
	chRegSetThreadName("speed_encoder_node");

	node.advertise(vel_pub, "encoder1");

	configure_tim4();

	for (;;) {
		time = chTimeNow();
		uint16_t delta = T2R(delta_tim4());

		if (vel_pub.alloc(msgp)) {
			msgp->delta = delta;
			vel_pub.publish(*msgp);
		}

		time += MS2ST(20);
		chThdSleepUntil(time);
	}

	return CH_SUCCESS;
}

/*===========================================================================*/
/* Application threads.                                                      */
/*===========================================================================*/

/*
 * Application entry point.
 */
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

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 1, r2p::ledsub_node, NULL);
	chThdCreateFromHeap(NULL, THD_WA_SIZE(1024), NORMALPRIO, steer_absolute_encoder_node, NULL);
	chThdCreateFromHeap(NULL, THD_WA_SIZE(1024), NORMALPRIO, speed_encoder_node, NULL);

	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
