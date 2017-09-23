/* #include <ros/ros.h> */

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include <linux/i2c-dev.h>


#define OFFSET 0x00 /* use 0x01 for 0.7V--2.7V) */

int file;
struct timespec sample_period = { 0, 100000000 };

void open_adc(int i2c_addr_adc) {
	file = open("/dev/i2c-0", O_RDWR);
	if (file < 0) {
		perror("failed to open file");
		exit(1);
	}
	if (ioctl(file, I2C_SLAVE_FORCE, i2c_addr_adc) < 0) {
		perror("ioctl failed");
		exit(2);
	}
}

void adc_set(char reg, char data) {
	char buf[2];
	buf[0] = reg;
	buf[1] = data;
	if (write(file, buf, 2) != 2) {
		fprintf(stderr, "i2c write transaction failed\n");
		exit(3);
	}
}

char adc_get(char reg) {
	char buf[1];
	adc_set(reg, 0);
	if (read(file, buf, 1) != 1) {
		fprintf(stderr, "i2c read transaction failed\n");
		exit(3);
	}
	return buf[0];
}


uint32_t enable_adc() {
	adc_set(0x83, 0x80); /* disable ADC in on GPIO0 */
	adc_set(0x90, 0x04); /* set GPIO0 to 12-bit ADC */
	adc_set(0x85, OFFSET); /* set range */
	adc_set(0x83, 0x88); /* enable ADC on GPIO0 */
}

uint32_t read_adc() {
	uint32_t r_high8 = adc_get(0x64);
	uint32_t r_low4 = adc_get(0x65);
	uint32_t val = r_high8 << 4 | r_low4;
	uint32_t millivolts = ((val * 1000) >> 11) + (OFFSET * 700);
	return millivolts;
}

int main(int argc, char** argv) {
	uint32_t millivolts = 0;
	open_adc(0x34);
	enable_adc();
	while (1) {
		millivolts = read_adc();
		printf("ADC value: %d millivolts\n", millivolts);
		nanosleep(&sample_period, NULL);
	}
	return 0;
}
