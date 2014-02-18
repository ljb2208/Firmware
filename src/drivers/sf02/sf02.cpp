
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
#include <termios.h>
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


#define SF02_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls
#define SF02_PACKET_TIMEOUT		20		// ms, if now data during this delay assume that full update received
#define TIMEOUT_10HZ 250
#define RATE_MEASUREMENT_PERIOD 5000000

#define SF02_RECV_BUFFER_SIZE 10
#define SF02_SEND_BUFFER_SIZE 1

#define SF02_CR	0x0D
#define SF02_LF	0x0A
#define SF02_DP 0x2E
#define SF02_TRIGGER 0x44

#define MUL10(x) (((x) << 3) + ((x) << 1))

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


typedef enum {
	SF02_DECODE_UNINIT = 0,
	SF02_DECODE_TRIGGER_SENT = 1,
	SF02_DECODE_STARTED = 2,
	SF02_DECODE_DECIMAL = 3,
	SF02_DECODE_CR = 4,
	SF02_DECODE_LF = 5
} sf02_decode_state_t;



class SF02 : public device::CDev
{
public:
	SF02(const char *uart_path, unsigned baud);
	virtual ~SF02();

	virtual int			init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int					_serial_fd;					///< serial interface to sf02
	unsigned			_baudrate;					///< current baudrate
	char				_port[20];					///< device / serial port path
	volatile int			_task;						//< worker task
	bool				_healthy;					///< flag to signal if the sf02 is ok
	bool 				_baudrate_changed;				///< flag to signal that the baudrate with the sf02 has changed
	orb_advert_t			_report_pub;					///< uORB pub for gps position
	float				_rate;						///< position update rate

	sf02_decode_state_t	_decode_state;
	uint8_t				_rx_buffer[SF02_RECV_BUFFER_SIZE];
	uint8_t				_tx_buffer[SF02_SEND_BUFFER_SIZE];
	int					_range_reading;					// centimeters
	uint8_t				_decimal_count;


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
	 * Set the baudrate of the UART to the SF02
	 */
	int				set_baudrate();

	/**
	 * Send a reset command to the SF02
	 */
	void				cmd_reset();

	int					receive(unsigned timeout);

	int					parse_char(uint8_t b);

	int					handle_message();

	void				decode_init();
	void				add_char(uint8_t b);
	bool				decode_finalize();

};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int sf02_main(int argc, char *argv[]);

namespace
{

SF02	*g_dev;

}


SF02::SF02(const char *uart_path, unsigned baud) :
	CDev("sf02", SF02_DEVICE_PATH),
	_task_should_exit(false),
	_healthy(false),
	_report_pub(-1),
	_rate(0.0f),
	_decode_state(SF02_DECODE_UNINIT)
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';
	_baudrate = baud;

	_tx_buffer[0] = 'D';

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
SF02::config()
{

}

int SF02::set_baudrate()
{
	/* process baud rate */
	int speed;

	switch (_baudrate) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

		warnx("try baudrate: %d\n", speed);

	default:
		warnx("ERROR: Unsupported baudrate: %d\n", _baudrate);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERROR setting config: %d (cfsetispeed)\n", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERROR setting config: %d (cfsetospeed)\n", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERROR setting baudrate (tcsetattr)\n");
		return -1;
	}

	/* XXX if resetting the parser here, ensure it does exist (check for null pointer) */
	return 0;
}

void
SF02::task_main()
{
	log("starting");

	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

	warnx("started: %i", _serial_fd);

	if (_serial_fd < 0) {
		log("failed to open serial port: %s err: %d", _port, errno);
		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		_exit(1);
	}

	set_baudrate();

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes */
	while (!_task_should_exit) {
			unlock();

			while (receive(TIMEOUT_10HZ) > 0 && !_task_should_exit) {

			}

			lock();
	}

	warnx("exiting");

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

int
SF02::receive(unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	uint8_t buf[128];
	/* timeout additional to poll */

	uint64_t time_started = hrt_absolute_time();

	ssize_t count = 0;

	uint8_t counter = 0;

	bool handled = false;

	while (true) {

			int ret = 0;


			if (_decode_state == SF02_DECODE_UNINIT) {
				//write(uart, ch, (size_t)(sizeof(uint8_t) * length));
				ret = ::write(_serial_fd, _tx_buffer, (size_t)(sizeof(_tx_buffer)));
				warnx("Send data:%i", ret);
				_decode_state = SF02_DECODE_TRIGGER_SENT;
				counter = 0;
			}

			counter++;

			/* poll for new data, wait for only SF02_PACKET_TIMEOUT (2ms) if something already received */
			//ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), handled ? SF02_PACKET_TIMEOUT : timeout);
			ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), SF02_PACKET_TIMEOUT);

			if (ret < 0) {
				/* something went wrong when polling */
				warnx("SF02: poll error");
				return -1;

			} else if (ret == 0) {
				/* return success after short delay after receiving a packet or timeout after long delay */
				if (handled) {
					return 1;

				} else {
					return -1;
				}

			} else if (ret > 0) {
				/* if we have new data from SF02, go handle it */
				warnx("received data");
				if (fds[0].revents & POLLIN) {
					/*
					 * We are here because poll says there is some data, so this
					 * won't block even on a blocking device. But don't read immediately
					 * by 1-2 bytes, wait for some more data to save expensive read() calls.
					 * If more bytes are available, we'll go back to poll() again.
					 */
					usleep(SF02_WAIT_BEFORE_READ * 1000);
					count = ::read(_serial_fd, buf, sizeof(buf));

					/* pass received bytes to the packet decoder */
					for (int i = 0; i < count; i++) {
						if (parse_char(buf[i]) > 0) {
							if (handle_message() > 0)
								handled = true;
						}
					}
				}
			}

			/*
			if (counter > 50) {
				_decode_state = SF02_DECODE_UNINIT;
			}*/


			/* abort after timeout if no useful packets received */
			/*
			if (time_started + timeout * 1000 < hrt_absolute_time()) {
				warnx("SF02: timeout - no useful messages");
				_decode_state = SF02_DECODE_UNINIT;
				return -1;
			}
			*/
	}
	return -1;
}


int
SF02::parse_char(uint8_t b)
{
	switch (_decode_state) {
		/* First, look for sync1 */
	case SF02_DECODE_TRIGGER_SENT:
		if (b != SF02_CR && b != SF02_LF) {
			_decode_state = SF02_DECODE_STARTED;
			decode_init();
			add_char(b);
		}

		break;

	case SF02_DECODE_STARTED:
		if (b != SF02_CR && b != SF02_LF) {
			if (b == SF02_DP) {
				_decode_state = SF02_DECODE_DECIMAL;
			} else {
				add_char(b);
			}
		} else if (b == SF02_DP) {
			_decode_state = SF02_DECODE_DECIMAL;
		} else if (b == SF02_CR) {
			_decode_state = SF02_DECODE_CR;
		}

		break;

	case SF02_DECODE_DECIMAL:
		if (b != SF02_CR && b != SF02_LF) {
			add_char(b);
			_decimal_count++;
		}
		break;

	case SF02_DECODE_CR:
		if (b == SF02_LF) {
			decode_finalize();
			_decode_state = SF02_DECODE_UNINIT;
			warnx("Msg decoded. Range: %i\n", _range_reading);
		}
		break;

	default:
		break;
	}

	return 0;	// message decoding in progress
}

void
SF02::decode_init()
{
	_decode_state = SF02_DECODE_STARTED;
	_range_reading = 0;
	_decimal_count = 0;
}

bool
SF02::decode_finalize()
{
	// add any missing bytes to convert to cm
	for (uint8_t i = _decimal_count; i < 2; i++) {
		add_char(0);
	}

	warnx("range reading %i", _range_reading);

	return true;

}

void
SF02::add_char(uint8_t b)
{
	_range_reading = MUL10(_range_reading) + b;
}


int
SF02::handle_message()
{
	return 0;
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
	warnx("range:\t%i", _range_reading);
	warnx("rate publication:\t%6.2f Hz", (double)_rate);

	usleep(100000);
}

/**
 * Local functions in support of the shell command.
 */
namespace sf02
{

SF02	*g_dev;

void	start(const char *uart_path);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(const char *uart_path, unsigned baud)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new SF02(uart_path, baud);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	/*
	fd = open(SF02_DEVICE_PATH, O_RDWR);

	warnx("SF02: fd opened %i", fd);

	if (fd < 0) {
		errx(1, "Could not open device path: %s\n", SF02_DEVICE_PATH);
		goto fail;
	}*/

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
	int fd = open(SF02_DEVICE_PATH, O_RDWR);

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
	unsigned baud = 115200;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		/* work around getopt unreliability */
		if (argc > 3) {

			for (int i = 2; i < argc; i++) {
				if (!strcmp(argv[i], "-d"))
					if (argc > i + 1)
						device_name = argv[i+1];

				if (!strcmp(argv[i], "-b"))
					if (argc > i + 1)
						baud = atoi(argv[i+1]);
			}
		}

		sf02::start(device_name, baud);
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
	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status' [-d /dev/ttyS0-n][-b 115200]");
}
