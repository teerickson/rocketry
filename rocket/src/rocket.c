/**
 * @file rc_test_mpu.c
 * @example    rc_test_mpu
 *
 * @brief      serves as an example of how to use the MPU in normal mode.
 *
 *             This does not use the DMP, for a demonstration of DMP
 *             functionality see rc_test_dmp.
 *
 * @author     James Strawson
 * @date       1/29/2018
 */


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <getopt.h>
#include <rc/mpu.h>
#include <rc/time.h>

extern char *optarg;

// i2C bus for Robotics Cape and Beaglebone Black is 2
#define I2C_BUS 2

static int enable_magnetometer = 1;
static int enable_thermometer = 1;
static int enable_warnings = 1;
static int running = 0;

static int grav_rad = 1;
static int grav_deg = 1;
static int grav_raw = 1;

static int accel_ms2 = 1;
static int accel_g = 1;
static int accel_raw = 1;

// printed if some invalid argument was given
static void __print_usage(void)
{
	printf("\n");
	printf("-a	print raw 16-bit ADC values for gyro & acceleration\n");
	printf("-d	print gyro in degrees/s\n");
	printf("-r	print gyro in radians/s\n");
	printf("-m	print acceleration in m/s^2\n");
	printf("-g	print acceleration in G\n");
	printf("-t [ms] set polling delay time (default 100 ms)\n");
	printf("-w	print i2c warnings\n");
	printf("-h	print this help message\n");
	printf("\n");
}

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main(int argc, char *argv[])
{
	rc_mpu_data_t data; //struct to hold new data
	int c;

	// How long to sleep each polling cycle.
	// TODO - make this an argument.
	int sleep_ms = 100;

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "adrgmt:wh")) != -1){
		switch (c){
		case 'a':
			grav_raw = 1;
			accel_raw = 1;
			printf("\nRaw values are from 16-bit ADC\n");
			break;
		case 'd':
			grav_deg = 1;
			break;
		case 'r':
			grav_rad = 1;
			break;
		case 'g':
			accel_g = 1;
			break;
		case 'm':
			accel_ms2 = 1;
			break;
		case 't':
			sleep_ms = atoi(optarg);
			break;
		case 'w':
			enable_warnings = 1;
			break;
		case 'h':
			__print_usage();
			return 0;
		default:
			__print_usage();
			return -1;
		}
	}

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	// use defaults for now, except also enable magnetometer.
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	conf.accel_fsr = ACCEL_FSR_16G;
	conf.enable_magnetometer = enable_magnetometer;
	conf.show_warnings = enable_warnings;

	if(rc_mpu_initialize(&data, conf)){
		fprintf(stderr,"rc_mpu_initialize_failed\n");
		return -1;
	}

	FILE* pLogFile = fopen("/home/debian/source/rocket/logs/mpu.log", "w");

	// print the header
	printf("\ntry 'rocket -h' to see other options\n\n");

	fprintf(pLogFile, "          Time          |");

	if(accel_ms2) {
		fprintf(pLogFile, "   Accel XYZ(m/s^2)  |");
	}
	if(accel_g) {
		fprintf(pLogFile,"     Accel XYZ(G)    |");
	}
	if(accel_raw) {
		fprintf(pLogFile, "  Accel XYZ(raw ADC) |");
	}
	
	if(grav_rad) {
		fprintf(pLogFile, "   Gyro XYZ (rad/s)  |");
	}
	if(grav_deg) {
		fprintf(pLogFile, "   Gyro XYZ (deg/s)  |");
	}
	if(grav_raw) {
		fprintf(pLogFile, "  Gyro XYZ (raw ADC) |");
	}

	if(enable_magnetometer)	fprintf(pLogFile, "  Mag Field XYZ(uT)  |");
	if(enable_thermometer) fprintf(pLogFile, " Temp (C)");
	fprintf(pLogFile, "\n");
	
	//now just wait, print_data will run
	while (running) {
		time_t t = time(NULL);	
        	struct tm tmNow = *localtime(&t);
		
		struct timeval tvNow;
		gettimeofday(&tvNow, NULL);
                int intMs = ( tvNow.tv_usec / 1000 ) % 1000;

		fprintf(pLogFile, "%d-%02d-%02d %02d:%02d:%02d.%03d |", tmNow.tm_year + 1900, tmNow.tm_mon + 1, tmNow.tm_mday, tmNow.tm_hour, tmNow.tm_min, tmNow.tm_sec, intMs);

		// read sensor data
		if(rc_mpu_read_accel(&data)<0){
			printf("read accel data failed\n");
		}
		if(rc_mpu_read_gyro(&data)<0){
			printf("read gyro data failed\n");
		}
		if(enable_magnetometer && rc_mpu_read_mag(&data)){
			printf("read mag data failed\n");
		}
		if(enable_thermometer && rc_mpu_read_temp(&data)){
			printf("read imu thermometer failed\n");
		}


		if(accel_ms2) {
			fprintf(pLogFile, "%6.2f %6.2f %6.2f |",\
							data.accel[0],\
							data.accel[1],\
							data.accel[2]);
		}
		if(accel_g) {
			fprintf(pLogFile, "%6.2f %6.2f %6.2f |",\
							data.accel[0]*MS2_TO_G,\
							data.accel[1]*MS2_TO_G,\
							data.accel[2]*MS2_TO_G);
		}
		if(accel_raw) {	
			fprintf(pLogFile, "%6d %6d %6d |",\
							data.raw_accel[0],\
							data.raw_accel[1],\
							data.raw_accel[2]);
		}

		if(grav_rad) {
			fprintf(pLogFile, "%6.1f %6.1f %6.1f |",\
							data.gyro[0]*DEG_TO_RAD,\
							data.gyro[1]*DEG_TO_RAD,\
							data.gyro[2]*DEG_TO_RAD);
		}
		if(grav_deg) {
			fprintf(pLogFile, "%6.1f %6.1f %6.1f |",\
							data.gyro[0],\
							data.gyro[1],\
							data.gyro[2]);
		}
		if(grav_raw) {
			fprintf(pLogFile, "%6d %6d %6d |",\
							data.raw_gyro[0],\
							data.raw_gyro[1],\
							data.raw_gyro[2]);
		}

		// read magnetometer
		if(enable_magnetometer){
			fprintf(pLogFile, "%6.1f %6.1f %6.1f |",\
							data.mag[0],\
							data.mag[1],\
							data.mag[2]);
		}
		// read temperature
		if(enable_thermometer){
			fprintf(pLogFile, " %4.2f    ", data.temp);
		}

		fprintf(pLogFile, "\n");

		rc_usleep(sleep_ms*1000);
	}
	printf("\n");

	fflush(pLogFile);
	fclose(pLogFile);
	rc_mpu_power_off();
	return 0;
}
