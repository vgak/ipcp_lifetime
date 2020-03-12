#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>

#include <gpib/ib.h>

// === config
#define HANTEK_TMC "/dev/usbtmc0"
#define PPS_GPIB_NAME "AKIP-1142/3G"

// === pps
#define PPS_TIMEOUT_S 1

// === osc
#define OSC_ACQUIRE_POINTS 64000
#define OSC_ACQUIRE_BLOCK_SIZE 4000
#define OSC_ACQUIRE_BLOCKS (OSC_ACQUIRE_POINTS / OSC_ACQUIRE_BLOCK_SIZE)
#define OSC_YSCALE_V 0.5 // 1, 2, 5 ...
#define OSC_HOLDOFF_S 1 // 1, 2, 5 ...
#define OSC_DUTY 50

#define OSC_TIMEOUT_S 2
#define OSC_ATTEMPTS 5

// === threads
void *commander(void *);
void *worker(void *);

// === utils
int get_run();
void set_run(int run_new);
double get_time();

int gpib_write(int fd, const char *str);
int gpib_read(int fd, char *buf, long len);
void gpib_print_error(int fd);

int usbtmc_write(int dev, const char *cmd);
int usbtmc_read(int dev, char *buf, int buf_length);
int usbtmc_print(int dev, const char *format, ...);

double freq_to_scale(int freq);


// === global variables
char dir_name[100];
pthread_rwlock_t run_lock;
int run;
const char *experiment_name;

// === program entry point
int main(int argc, char const *argv[])
{
	int ret = 0;
	int status;

	time_t start_time;
	struct tm start_time_struct;

	pthread_t t_commander;
	pthread_t t_worker;

	// === check input parameters
	if (argc < 2)
	{
		fprintf(stderr, "# E: Usage: vac <experiment_name>\n");
		ret = -1;
		goto main_exit;
	}
	experiment_name = argv[1];

	// === get start time of experiment
	start_time = time(NULL);
	localtime_r(&start_time, &start_time_struct);

	// === we need actual information w/o buffering
	setlinebuf(stdout);
	setlinebuf(stderr);

	// === initialize run state variable
	pthread_rwlock_init(&run_lock, NULL);
	run = 1;

	// === create dirictory in "20191012_153504_<experiment_name>" format
	snprintf(dir_name, 100, "%04d-%02d-%02d_%02d-%02d-%02d_%s",
		start_time_struct.tm_year + 1900,
		start_time_struct.tm_mon + 1,
		start_time_struct.tm_mday,
		start_time_struct.tm_hour,
		start_time_struct.tm_min,
		start_time_struct.tm_sec,
		experiment_name
	);
	status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (status == -1)
	{
		fprintf(stderr, "# E: unable to create experiment directory (%s)\n", strerror(errno));
		ret = -2;
		goto main_exit;
	}

	// === now start threads
	pthread_create(&t_commander, NULL, commander, NULL);
	pthread_create(&t_worker, NULL, worker, NULL);

	// === and wait ...
	pthread_join(t_worker, NULL);

	// === cancel commander thread becouse we don't need it anymore
	// === and wait for cancelation finish
	pthread_cancel(t_commander);
	pthread_join(t_commander, NULL);
	printf("\n");

	main_exit:
	return ret;
}

// === commander function
void *commander(void *arg)
{
	(void) arg;

	char str[100];
	char *s;
	int ccount;

	while(get_run())
	{
		printf("> ");

		s = fgets(str, 100, stdin);
		if (s == NULL)
		{
			fprintf(stderr, "# E: Exit\n");
			set_run(0);
			break;
		}

		switch(str[0])
		{
			case 'h':
				printf(
					"Help:\n"
					"\th -- this help;\n"
					"\tq -- exit the program;\n");
				break;
			case 'q':
				set_run(0);
				break;
			default:
				ccount = strlen(str)-1;
				fprintf(stderr, "# E: Unknown command (%.*s)\n", ccount, str);
				break;
		}
	}

	return NULL;
}

// === worker function
void *worker(void *arg)
{
	(void) arg;

	int r;

	int osc_fd;
	int pps_fd;

	char filename_lt[100];
	char gnuplot_cmd[100];
	FILE *lt_fp;
	FILE *gp;

	// frequencies and corresponding scales
	double freq[10] = {1000, 500, 200, 100, 50, 20, 10, 5, 2, 1};
	double scale[10];
	for (int i = 0; i < 10; ++i) scale[i] = freq_to_scale(freq[i]);

	// === first we are connecting to instruments
	r = open(HANTEK_TMC, O_RDWR);
	if(r == -1)
	{
		fprintf(stderr, "# E: Unable to open hantek (%s)\n", strerror(errno));
		goto worker_open_hantek;
	}
	osc_fd = r;

	r = ibfind(PPS_GPIB_NAME);
	if(r == -1)
	{
		fprintf(stderr, "# E: Unable to open power supply (%d)\n", r);
		goto worker_pps_ibfind;
	}
	pps_fd = r;

	// === init pps
	gpib_write(pps_fd, "output 0");
	gpib_write(pps_fd, "instrument:nselect 1");
	gpib_write(pps_fd, "voltage:limit 11V");
	gpib_write(pps_fd, "voltage 10.0");
	gpib_write(pps_fd, "current 0.1");
	gpib_write(pps_fd, "channel:output 1");
	gpib_write(pps_fd, "instrument:nselect 2");
	gpib_write(pps_fd, "voltage:limit 5.5V");
	gpib_write(pps_fd, "voltage 5.0");
	gpib_write(pps_fd, "current 0.15");
	gpib_write(pps_fd, "channel:output 1");
	gpib_write(pps_fd, "instrument:nselect 1");

	sleep(PPS_TIMEOUT_S);

	// === init osc
	usbtmc_write(osc_fd, "dds:switch 0");
	usbtmc_write(osc_fd, "channel1:display off");
	usbtmc_write(osc_fd, "channel2:display off");
	usbtmc_write(osc_fd, "channel3:display off");
	usbtmc_write(osc_fd, "channel4:display off");
	usbtmc_write(osc_fd, "channel1:bwlimit on");
	usbtmc_write(osc_fd, "channel1:coupling dc");
	usbtmc_write(osc_fd, "channel1:invert off");
	usbtmc_print(osc_fd, "channel1:scale %fV", OSC_YSCALE_V);
	usbtmc_write(osc_fd, "channel1:offset 0V");
	usbtmc_write(osc_fd, "channel1:probe 1");
	usbtmc_write(osc_fd, "channel1:vernier off");
	usbtmc_write(osc_fd, "channel1:display on");
	usbtmc_write(osc_fd, "acquire:type normal");
	usbtmc_print(osc_fd, "acquire:points %d", OSC_ACQUIRE_POINTS);
	usbtmc_write(osc_fd, "timebase:mode main");
	usbtmc_write(osc_fd, "timebase:vernier off");
	usbtmc_print(osc_fd, "timebase:scale %.5lf", scale[0]); // = 1, 2 ,5 ...
	usbtmc_write(osc_fd, "trigger:mode edge");
	usbtmc_write(osc_fd, "trigger:sweep auto");
	usbtmc_print(osc_fd, "trigger:holdoff %d", OSC_HOLDOFF_S);
	usbtmc_write(osc_fd, "trigger:edge:source ext");
	usbtmc_write(osc_fd, "trigger:edge:slope rising");
	usbtmc_write(osc_fd, "trigger:edge:level 1.75");
	usbtmc_write(osc_fd, "dds:type square");
	usbtmc_print(osc_fd, "dds:freq %.0lf", freq[0]);
	usbtmc_write(osc_fd, "dds:amp 3.5");
	usbtmc_write(osc_fd, "dds:offset 1.75");
	usbtmc_print(osc_fd, "dds:duty %d", OSC_DUTY);
	usbtmc_write(osc_fd, "dds:wave:mode off");
	usbtmc_write(osc_fd, "dds:burst:switch off");
	usbtmc_write(osc_fd, "dds:switch 1");

	sleep(OSC_TIMEOUT_S);

	// === create log file
	snprintf(filename_lt, 100, "%s/lifetime.dat", dir_name);

	lt_fp = fopen(filename_lt, "w+");
	if(lt_fp == NULL)
	{
		fprintf(stderr, "# E: Unable to open file \"%s\" (%s)\n", filename_lt, strerror(ferror(lt_fp)));
		goto worker_vac_fopen;
	}
	setlinebuf(lt_fp);

	// === write vac header
	r = fprintf(lt_fp,
		"# mipt_r125_lifetime\n"
		"# Dependence of decay curve on frequency\n"
		"# Experiment name \"%s\"\n"
		"#\n"
		"# Columns:\n"
		"# 1 - index\n"
		"# 2 - time, s\n"
		"# 3 - sample voltage, V\n"
		"# 4 - sample current, A\n"
		"# 5 - laser voltage, V\n"
		"# 6 - laser current, A\n"
		"# 7 - laset modulation rate, Hz\n"
		"# 8 - laset modulation duty, %%\n"
		"# 9 - curve sampling rate, Hz\n",
		experiment_name
	);
	if(r < 0)
	{
		fprintf(stderr, "# E: Unable to print to file \"%s\" (%s)\n", filename_lt, strerror(r));
		goto worker_lt_header;
	}

	// === open gnuplot
	snprintf(gnuplot_cmd, 100, "gnuplot > %s/gnuplot.log 2>&1", dir_name);
	gp = popen(gnuplot_cmd, "w");
	if (gp == NULL)
	{
		fprintf(stderr, "# E: unable to open gnuplot pipe (%s)\n", strerror(errno));
		goto worker_gp_popen;
	}
	setlinebuf(gp);

	// === prepare gnuplot
	r = fprintf(gp,
		"set xlabel \"Time, s\"\n"
		"set ylabel \"Voltage, V\"\n"
	);
	if(r < 0)
	{
		fprintf(stderr, "# E: Unable to print to gp (%s)\n", strerror(r));
		goto worker_gp_settings;
	}

	// === let the action begins!
	for (int lt_index = 0; lt_index < 10; ++lt_index)
	{
		double lt_time;
		double sample_voltage;
		double sample_current;
		double laser_voltage;
		double laser_current;
		double sampling_rate;
		char   buf[100];

		if (get_run())
		{
			usbtmc_print(osc_fd, "dds:freq %.0lf", freq[lt_index]);
			usbtmc_print(osc_fd, "timebase:scale %.5lf", scale[lt_index]);
			sleep(OSC_TIMEOUT_S);

			lt_time = get_time();
			if (lt_time < 0)
			{
				fprintf(stderr, "# E: Unable to get time\n");
				set_run(0);
				break;
			}

			gpib_write(pps_fd, "measure:voltage:all?");
			gpib_read(pps_fd, buf, 100);
			sscanf(buf, "%lf, %lf", &sample_voltage, &laser_voltage);

			gpib_write(pps_fd, "measure:current:all?");
			gpib_read(pps_fd, buf, 100);
			sscanf(buf, "%lf, %lf", &sample_current, &laser_current);

			for (int attempt = 0; attempt < OSC_ATTEMPTS; ++attempt)
			{
				FILE *curve_fp;
				char *b;
				char filename_curve[100];
				char buf_curve_head[128];
				char buf_curve_body[4029];
				char srbuf[10] = {0};

				if (get_run())
				{
					// === crate curve file
					snprintf(filename_curve, 100, "%s/curve_%d.%d.dat", dir_name, lt_index, attempt);
					curve_fp = fopen(filename_curve, "w+");
					if(curve_fp == NULL)
					{
						fprintf(stderr, "# E: Unable to open file \"%s\" (%s)\n", filename_curve, strerror(ferror(curve_fp)));
						set_run(0);
						continue;
					}
					setlinebuf(curve_fp);

					// === write curve header
					r = fprintf(curve_fp,
						"# mipt_r125_lifetime\n"
						"# Dependence of decay curve on frequency\n"
						"# Experiment name \"%s\"\n"
						"# Columns:\n"
						"# 1 - index, points\n"
						"# 2 - value, int8_t\n",
						experiment_name
					);

					usbtmc_write(osc_fd, "trigger:sweep single");
					usbtmc_write(osc_fd, "trigger:force");
					usleep((16 * OSC_ACQUIRE_BLOCKS * scale[lt_index] + OSC_HOLDOFF_S) * 1e6);

					usbtmc_write(osc_fd, "waveform:data:all?");
					usbtmc_read(osc_fd, buf_curve_head, 128);

					b = buf_curve_head;
					fprintf(curve_fp, "# packet head = %.2s\n", b); b += 2;
					fprintf(curve_fp, "# Represents the byte length of the current packet = %.9s\n", b); b += 9;
					fprintf(curve_fp, "# The total length of bytes representing the amount of data = %.9s\n", b); b += 9;
					fprintf(curve_fp, "# Represents the byte length of the data that has been uploaded = %.9s\n", b); b += 9;
					fprintf(curve_fp, "# Represents the current running state = %.1s\n", b); b += 1;
					fprintf(curve_fp, "# Represents the state of the trigger = %.1s\n", b); b += 1;
					fprintf(curve_fp, "# bias of ch1 = %.4s\n", b); b += 4;
					fprintf(curve_fp, "# bias of ch2 = %.4s\n", b); b += 4;
					fprintf(curve_fp, "# bias of ch3 = %.4s\n", b); b += 4;
					fprintf(curve_fp, "# bias of ch4 = %.4s\n", b); b += 4;
					fprintf(curve_fp, "# volt of ch1 = %.8s\n", b); b += 8;
					fprintf(curve_fp, "# volt of ch2 = %.8s\n", b); b += 8;
					fprintf(curve_fp, "# volt of ch3 = %.8s\n", b); b += 8;
					fprintf(curve_fp, "# volt of ch4 = %.8s\n", b); b += 8;
					fprintf(curve_fp, "# Represents the enabling of the channel [1-4] = %.4s\n", b); b += 4;
					memcpy(srbuf, b, 9);
					sampling_rate = atof(srbuf);
					fprintf(curve_fp, "# Indicated sampling rate = %.9s\n", b); b += 9;
					fprintf(curve_fp, "# Sampling multiple = %.6s\n", b); b += 6;
					fprintf(curve_fp, "# The current frame displays trigger time = %.9s\n", b); b += 9;
					fprintf(curve_fp, "# The current frame shows the starting point of the data start point = %.9s\n", b); b += 9;
					fprintf(curve_fp, "# Reserved bit = %.12s\n", b); b += 12;

					for (int block = 0; block < OSC_ACQUIRE_BLOCKS; ++block)
					{
						usbtmc_write(osc_fd, "waveform:data:all?");
						usbtmc_read(osc_fd, buf_curve_body, 4029);

						b = buf_curve_body;
						fprintf(curve_fp, "# packet head = %.2s\n", b); b += 2;
						fprintf(curve_fp, "# Represents the byte length of the current packet = %.9s\n", b); b += 9;
						fprintf(curve_fp, "# The total length of bytes representing the amount of data = %.9s\n", b); b += 9;
						fprintf(curve_fp, "# Represents the byte length of the data that has been uploaded = %.9s\n", b); b += 9;
						for (int block_point = 0; block_point < OSC_ACQUIRE_BLOCK_SIZE; ++block_point)
						{
							fprintf(curve_fp, "%d\t%d\n", block * OSC_ACQUIRE_BLOCK_SIZE + block_point, b[block_point]);
						}
					}

					r = fprintf(gp,
						"set title \"i = %d.%d, t = %.3lf s, U = %.3lf V, I = %.3lf A, Ul = %.3lf V, Il = %.3lf A\"\n"
						"plot \"%s\" u ($1/%le):($2*%le) w l lw 1 title \"freq = %.1lf Hz, duty = %d %%\"\n",
						lt_index, attempt, lt_time, sample_voltage, sample_current, laser_voltage, laser_current,
						filename_curve, sampling_rate, 10*OSC_YSCALE_V/256.0, freq[lt_index], OSC_DUTY
					);
					if(r < 0)
					{
						fprintf(stderr, "# E: Unable to print to gp (%s)\n", strerror(r));
						set_run(0);
					}

					r = fclose(curve_fp);
					if (r == EOF)
					{
						fprintf(stderr, "# E: Unable to close file \"%s\" (%s)\n", filename_curve, strerror(errno));
					}
				}
				else
				{
					break;
				}
			}

			r = fprintf(lt_fp, "%d\t%le\t%.3le\t%.3le\t%.3le\t%.3le\t%.0lf\t%d\t%le\n",
				lt_index,
				lt_time,
				sample_voltage,
				sample_current,
				laser_voltage,
				laser_current,
				freq[lt_index],
				OSC_DUTY,
				sampling_rate
			);
			if(r < 0)
			{
				fprintf(stderr, "# E: Unable to print to file \"%s\" (%s)\n", filename_lt, strerror(r));
				set_run(0);
				break;
			}
		}
		else
		{
			break;
		}
	}

	gpib_write(pps_fd, "output 0");
	gpib_write(pps_fd, "voltage 0");

	usbtmc_write(osc_fd, "dds:switch 0");
	usbtmc_write(osc_fd, "dds:offset 0");

	gpib_write(pps_fd, "system:beeper");

	worker_gp_settings:

	r = pclose(gp);
	if (r == -1)
	{
		fprintf(stderr, "# E: Unable to close gnuplot pipe (%s)\n", strerror(errno));
	}
	worker_gp_popen:


	worker_lt_header:

	r = fclose(lt_fp);
	if (r == EOF)
	{
		fprintf(stderr, "# E: Unable to close file \"%s\" (%s)\n", filename_lt, strerror(errno));
	}
	worker_vac_fopen:

	ibclr(pps_fd);
	gpib_write(pps_fd, "*rst");
	sleep(PPS_TIMEOUT_S);
	ibloc(pps_fd);
	worker_pps_ibfind:

	r = close(osc_fd);
	if(r == -1)
	{
		fprintf(stderr, "# E: Unable to close hantek (%s)\n", strerror(errno));
	}
	worker_open_hantek:

	return NULL;
}

// === utils
int get_run()
{
	int run_local;
	pthread_rwlock_rdlock(&run_lock);
		run_local = run;
	pthread_rwlock_unlock(&run_lock);
	return run_local;
}

void set_run(int run_new)
{
	pthread_rwlock_wrlock(&run_lock);
		run = run_new;
	pthread_rwlock_unlock(&run_lock);
}

double get_time()
{
	static int first = 1;
	static struct timeval t_first = {0};
	struct timeval t = {0};
	double ret;
	int r;

	if (first == 1)
	{
		r = gettimeofday(&t_first, NULL);
		if (r == -1)
		{
			fprintf(stderr, "# E: unable to get time (%s)\n", strerror(errno));
			ret = -1;
		}
		else
		{
			ret = 0.0;
			first = 0;
		}
	}
	else
	{
		r = gettimeofday(&t, NULL);
		if (r == -1)
		{
			fprintf(stderr, "# E: unable to get time (%s)\n", strerror(errno));
			ret = -2;
		}
		else
		{
			ret = (t.tv_sec - t_first.tv_sec) * 1e6 + (t.tv_usec - t_first.tv_usec);
			ret /= 1e6;
		}
	}

	return ret;
}


int gpib_write(int fd, const char *str)
{
	return ibwrt(fd, str, strlen(str));
}

int gpib_read(int fd, char *buf, long len)
{
	int r;

	r = ibrd(fd, buf, len);
	if (ibcnt < len)
	{
		buf[ibcnt] = 0;
	}

	return r;
}

void gpib_print_error(int fd)
{
	char buf[100] = {0};
	gpib_write(fd, "system:error?");
	gpib_read(fd, buf, 100);
	fprintf(stderr, "[debug] error = %s\n", buf);
}

int usbtmc_write(int dev, const char *cmd)
{
	int r;

	r = write(dev, cmd, strlen(cmd));
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to write to hantek (%s)\n", strerror(errno));
	}

	return r;
}

int usbtmc_read(int dev, char *buf, int buf_length)
{
	int r;

	r = read(dev, buf, buf_length);
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to read from hantek (%s)\n", strerror(errno));
	}

	return r;
}

int usbtmc_print(int dev, const char *format, ...)
{
	int r;
    va_list args;

    va_start(args, format);

    r = vdprintf(dev, format, args);
    if (r < 0)
	{
		fprintf(stderr, "# E: unable to printf to hantek (%s)\n", strerror(errno));
	}

    va_end(args);

    return r;
}

double freq_to_scale(int freq)
{
	// 2 * period < 16 * 16 * scale, scale = 1, 2, 5 ...
	double l, f, m, e;

	l = log10(2.0 * (1.0 / freq) / (16.0 * OSC_ACQUIRE_BLOCKS));
	f = fmod(l, 1.0);
	m = pow(10.0, f);
	e = pow(10.0, l - f);

	if      (m <= 0.1) m = 0.1;
	else if (m <= 0.2) m = 0.2;
	else if (m <= 0.5) m = 0.5;
	else if (m <= 1.0) m = 1.0;
	else if (m <= 2.0) m = 2.0;
	else if (m <= 5.0) m = 5.0;
	else               m = 10.0;

	return m * e;
}
