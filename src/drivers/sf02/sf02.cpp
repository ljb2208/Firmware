
/**
 * @file sf02.cpp
 * Driver for the sf02 on a serial port
 */

#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <drivers/drv_sf02.h>
//#include <uORB/topics/vehicle_gps_position.h>


#define TIMEOUT_5HZ 500
#define RATE_MEASUREMENT_PERIOD 5000000

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif



class SF02 : public device::CDev
{
public:
	SF02(const char *uart_path);
	virtual ~SF02();

	virtual int			init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int				_serial_fd;					///< serial interface to sf02
	unsigned			_baudrate;					///< current baudrate
	char				_port[20];					///< device / serial port path
	volatile int			_task;						//< worker task
	bool				_healthy;					///< flag to signal if the sf02 is ok
	bool 				_baudrate_changed;				///< flag to signal that the baudrate with the sf02 has changed
	orb_advert_t			_report_pub;					///< uORB pub for gps position
	float				_rate;						///< position update rate


	/**
	 * Try to configure the SF02
	 */
	void				config();

	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main(void);

	/**
	 * Set the baudrate of the UART to the GPS
	 */
	int				set_baudrate(unsigned baud);

	/**
	 * Send a reset command to the GPS
	 */
	void				cmd_reset();

};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int sf02_main(int argc, char *argv[]);

namespace
{

SF02	*g_dev;

}


SF02::SF02(const char *uart_path) :
	CDev("sf02", SF02_DEVICE_PATH),
	_task_should_exit(false),
	_healthy(false),
	_report_pub(-1),
	_rate(0.0f)
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* we need this potentially before it could be set in task_main */
	g_dev = this;

	_debug_enabled = true;
}

SF02::~SF02()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);

	g_dev = nullptr;

}

int
SF02::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK)
		goto out;

	/* start the SF02 driver worker task */
	_task = task_create("sf02", SCHED_PRIORITY_SLOW_DRIVER, 2048, (main_t)&SF02::task_main_trampoline, nullptr);

	if (_task < 0) {
		warnx("task start failed: %d", errno);
		return -errno;
	}

	ret = OK;
out:
	return ret;
}

int
SF02::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	lock();

	int ret = OK;

	switch (cmd) {
	case SENSORIOCRESET:
		cmd_reset();
		break;
	}

	unlock();

	return ret;
}

void
SF02::task_main_trampoline(void *arg)
{
	g_dev->task_main();
}

void
SF02::task_main()
{
	log("starting");

	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR);

	if (_serial_fd < 0) {
		log("failed to open serial port: %s err: %d", _port, errno);
		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		_exit(1);
	}

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {
			unlock();
			lock();
	}

	warnx("exiting");

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}



void
SF02::cmd_reset()
{
	//XXX add reset?
}

void
SF02::print_info()
{
	warnx("port: %s, baudrate: %d, status: %s", _port, _baudrate, (_healthy) ? "OK" : "NOT OK");
	warnx("rate publication:\t%6.2f Hz", (double)_rate);

	usleep(100000);
}

/**
 * Local functions in support of the shell command.
 */
namespace sf02
{

SF02	*g_dev;

void	start(const char *uart_path, bool fake_gps);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(const char *uart_path)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new SF02(uart_path);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(SF02_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		errx(1, "Could not open device path: %s\n", SF02_DEVICE_PATH);
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev;
	g_dev = nullptr;

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(SF02_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	exit(0);
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	g_dev->print_info();

	exit(0);
}

} // namespace


int
sf02_main(int argc, char *argv[])
{

	/* set to default */
	char *device_name = SF02_DEFAULT_UART_PORT;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		/* work around getopt unreliability */
		if (argc > 3) {
			if (!strcmp(argv[2], "-d")) {
				device_name = argv[3];

			} else {
				goto out;
			}
		}

		sf02::start(device_name);
	}

	if (!strcmp(argv[1], "stop"))
		sf02::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		sf02::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		sf02::reset();

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status"))
		sf02::info();

out:
	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status' [-d /dev/ttyS0-n][-f]");
}
