#include "ch.h"
#include "hal.h"

#include "adns9500.h"

static uint8_t txbuf[2];
static uint8_t rxbuf[2];

// TODO: acquire
data_t read(SPIDriver *spip, rreg_t address) {

	txbuf[0] = address;
	txbuf[1] = 0x00;

	spiSelect(spip);
	halPolledDelay(US2RTT(1));
	spiExchange(spip, 2, txbuf, rxbuf);
	halPolledDelay(US2RTT(1));
	spiUnselect(spip);

	return rxbuf[1];
}

// TODO: acquire
void write(SPIDriver *spip, wreg_t address, data_t data) {

	txbuf[0] = 0x80 | address;
	txbuf[1] = data;

	spiSelect(spip);
	halPolledDelay(US2RTT(1));
	spiSend(spip, 2, txbuf);
	halPolledDelay(US2RTT(20));
	spiUnselect(spip);
}

// TODO: acquire
void writen(SPIDriver *spip, wreg_t address, data_t * datap, uint16_t n) {

	txbuf[0] = 0x80 | address;

	spiSelect(spip);
	halPolledDelay(US2RTT(1));
	spiSend(spip, 1, txbuf);
	while (n--) {
		chThdSleepMilliseconds(2);
		spiSend(spip, 1, datap++);
	}
	halPolledDelay(US2RTT(20));
	spiUnselect(spip);
}

/*
 * ADNS-9500 thread.
 */
const SPIConfig spi2cfg = { NULL, /* HW dependent part.*/GPIOB, 12, SPI_CR1_BR_2 |  SPI_CR1_BR_1 | SPI_CR1_CPOL
		| SPI_CR1_CPHA };

msg_t adns_thread(void *arg) {
	SPIDriver * spip = (SPIDriver *) arg;
	uint8_t sid = 123;

	(void) arg;

	chRegSetThreadName("adns");

	spiStart(&SPI_DRIVER, &spi2cfg);

	chThdSleepMilliseconds(100);

	while (1) {
		palSetPad(LED2_GPIO, LED2);
		palSetPad(LED4_GPIO, LED4);

		palClearPad(VINH_GPIO, VINH);
		chThdSleepMilliseconds(500);
		palSetPad(VINH_GPIO, VINH);
		chThdSleepMilliseconds(1000);


		spiSelect(spip);
		halPolledDelay(US2RTT(20));
		spiUnselect(spip);
		halPolledDelay(US2RTT(20));
		spiSelect(spip);
		halPolledDelay(US2RTT(20));

		sid = read(spip, PRODUCT_ID);
		sid = read(spip, revision_id);

		write(spip, power_up_reset, 0x5a); // force reset
		halPolledDelay(MS2RTT(100));

		read(spip, motion);
		halPolledDelay(US2RTT(20));

		read(spip, delta_x_l);
		halPolledDelay(US2RTT(20));

		read(spip, delta_x_h);
		halPolledDelay(US2RTT(20));

		read(spip, delta_y_l);
		halPolledDelay(US2RTT(20));

		read(spip, delta_y_h);
		halPolledDelay(US2RTT(20));

		//sid = read(spip, PRODUCT_ID);
		//sid = read(spip, revision_id);

		write(spip, configuration_iv, 0x02);
		halPolledDelay(US2RTT(20));
		write(spip, srom_enable, 0x1d);
		halPolledDelay(US2RTT(1000));
		write(spip, srom_enable, 0x18);
		halPolledDelay(US2RTT(120));

//	writen(spip, srom_load_burst, srom, sizeof(srom));

		txbuf[0] = 0x80 | srom_load_burst;

		unsigned int n = 0;
		spiSelect(spip);
		spiSend(spip, 1, txbuf);
		halPolledDelay(US2RTT(20));
		while (n < sizeof(srom)) {
			spiSend(spip, 1, &(srom[n]));
			halPolledDelay(US2RTT(20));
			n++;
		}
		spiUnselect(spip);

		halPolledDelay(US2RTT(160));

//		sid = read(spip, PRODUCT_ID);
//		sid = read(spip, revision_id);
		sid = read(spip, srom_id);

		if (sid == 0x91)
			palClearPad(LED2_GPIO, LED2);
		else
			palClearPad(LED4_GPIO, LED4);

		sid = read(spip, PRODUCT_ID);
		sid = read(spip, revision_id);

		chThdSleepMilliseconds(2000);

	}

	return 0;
}
