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
#include <jasadc/ToggleGpio.h>

#define OFFSET 0 /* use 1 for 0.7V--2.7V) */
#define GPIO_MAX 8

int file;
struct timespec sample_period = { 0, 100000000 };
int gpio_state[GPIO_MAX];
int gpio_fd[GPIO_MAX];
char* gpio_fn[GPIO_MAX * 2];

/* ADC */

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
	return r_high8 << 4 | r_low4;
}

uint32_t convert_adc_to_microvolts(uint32_t adc) {
	return ((adc * 1000000) >> 11) + (OFFSET * 700000);
}


/* GPIO */

#include "get_xio_base.hpp"

void gpio_init() {
	char* buf;
	char* buf_clone;
	int i = 0;
	int export_fd = open("/sys/class/gpio/export", O_WRONLY);
	char* gpio;
	gpio[0] = 0;
	for (i = 0; i < GPIO_MAX; i++) {
		gpio = itoa(get_xio_base() + i);
		write(export_fd, gpio, strlen(gpio));
		/* get all filenames ahead of time */
		buf = (char*) malloc(100);
		buf_clone = (char*) malloc(100);
		strcat(buf, "/sys/class/gpio/gpio");
		strcat(buf, gpio);
		strcat(buf, "/");
		strcpy(buf_clone, buf);
		strcat(buf, "direction");
		gpio_fn[i + i + 0] = buf;
		strcat(buf_clone, "value");
		gpio_fn[i + i + 1] = buf_clone;
	}
	close(export_fd);
}

void gpio_uninit() {
	int i = 0;
	int export_fd = open("/sys/class/gpio/unexport", O_WRONLY);
	char* gpio;
	gpio[0] = 0;
	for (i = 0; i < GPIO_MAX; i++) {
		gpio = itoa(get_xio_base() + i);
		write(export_fd, gpio, strlen(gpio));
	}
	close(export_fd);
}

bool gpio_get(int pin) {
	char buf[1];
	if (read(gpio_fd[pin], buf, 1) != 1) {
		fprintf(stderr, "GPIO read failed\n");
		exit(33);
	}
	return buf[0] == '1';
}

void gpio_set(int pin, bool value) {
	char buf[1];
	buf[0] = '0' + value;
	if (write(gpio_fd[pin], buf, 1) != 1) {
		fprintf(stderr, "GPIO write failed; pin = %d, value = %d\n", pin, value);
		exit(34);
	}
}

bool serve_gpio(jasadc::ToggleGpio::Request& req,
                jasadc::ToggleGpio::Response& res) {
	using namespace jasadc::ToggleGpio;
	int pin = req.pin;
	int mode = req.mode;
	if (pin < 0 || pin >= GPIO_MAX ) {
		goto fail;
	}
	switch (mode) {
	case OUT_LOW:
		if (gpio_state[pin] != OUT_LOW) {
			if (gpio_state[pin] == OUT_HIGH) {
				gpio_set(pin, 0);
			} else {
				close(gpio_fd[pin]);
				gpio_fd[pin] = open(FIXME);
			}
		}
		break;
	case OUT_HIGH:

		break;
	case IN_PUB:
	case IN_NOPUB:
		
		break;
	default:
		goto fail;
	}
	gpio_state[pin] = mode;
	return true;
		fail:
	res.success = false;
	return true;
}

/* other */

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

/*
 * First argument is sample period in nanoseconds (optional; defaults to 0.1s).
 */
int main(int argc, char** argv) {
	uint32_t microvolts = 0;
	uint32_t adc_raw = 0;
	char* mac = get_mac("wlan0");
	char buf[128];
	int i = 17; /* length of MAC */

	/* parse args */
	if (argc >= 2) {
		sample_period.tv_nsec = atoi(argv[1]);
	}

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
	ros::Publisher pub_muv = n.advertise<std_msgs::Int32>("microvolts", 1);
	ros::Publisher pub_adc = n.advertise<std_msgs::Int32>("adc", 1);
	ros::ServiceServer n.advertiseService("gpio", serve_gpio);
	ros::Publisher* pub_gpios[GPIO_MAX];
	for (i = 0; i < GPIO_MAX; i++) {
		pub_gpios[i] = &n.advertise<std_msgs::Bool>("gpio" + i, 1); 
	}
	
	std_msgs::Int32 muv_output;
	std_msgs::Int32 adc_output;
	std_msgs::Bool gpio_output;
	muv_output.data = 0;
	adc_output.data = 0;
	gpio_output.data = false;

	open_adc(0x34);
	enable_adc();
	while (1) {
		adc_raw = read_adc();
		microvolts = convert_adc_to_microvolts(adc_raw);

		muv_output.data = microvolts;
		pub_muv.publish(muv_output);
		adc_output.data = adc_raw;
		pub_adc.publish(adc_output);

		for (i = 0; i < GPIO_MAX; i++) {
			if (gpio_state[i] == jasadc::ToggleGpio::IN_PUB) {
				gpio_output.data = gpio_get(i);
				pub_gpios[i]->publish(gpio_output);
			}
		}

		nanosleep(&sample_period, NULL);
	}
	return 0;
}
