#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include <linux/i2c-dev.h>


#include <ros/ros.h>
#include <std_msgs/Int32.h>


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


char* get_mac(const char* iface) {
	const int len_mac = 18;
	int fd;
	char fn[100];
	char* mac = (char*) malloc(len_mac);
	fn[0] = 0;
	strcat(fn, "/sys/class/net/");
	strcat(fn, iface);
	strcat(fn, "/address");
	fd = open(fn, O_RDONLY);
	if (fd < 0) {
		perror(fn);
		exit(16);
	}
	if (read(fd, mac, len_mac) != len_mac) {
		perror(mac);
		exit(17);
	}
	return mac;
}


int main(int argc, char** argv) {
	uint32_t millivolts = 0;
	char* mac = get_mac("wlan0");
	char buf[128];
	int i = 17; /* length of MAC */

	mac[i] = 0; /* remove newline */
	while (i-->0) {
		if (mac[i] == ':') {
			mac[i] = '_';
		}
	}
	buf[0] = 0;
	strcat(buf, "chip_adc_");
	strcat(buf, mac);
	ros::init(argc, argv, buf);
	ros::NodeHandle n("~");
	ros::Publisher pub = n.advertise<std_msgs::Int32>("millivolts/0", 1);
	std_msgs::Int32 adc_output;
	adc_output.data = 0;

	open_adc(0x34);
	enable_adc();
	while (1) {
		millivolts = read_adc();
		printf("ADC value: %d millivolts\n", millivolts);

		adc_output.data = millivolts;
		pub.publish(adc_output);

		nanosleep(&sample_period, NULL);
	}
	return 0;
}
