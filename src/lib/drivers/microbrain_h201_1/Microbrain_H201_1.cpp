

#include "Microbrain_H201_1.hpp"


Microbrain_H201_1::Microbrain_H201_1(const char* module_name, const char* port, uint8_t rotation, float_t min_range, float_t max_range, float_t h_fov, float_t v_fov):ScheduledWorkItem(module_name, px4::serial_port_to_wq(port)),
_px4_rangefinder(0, rotation),_fd(-1)
{

 	// store port name
	strncpy(_port, port, sizeof(_port) - 1);
	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	_rotation = rotation;
	_min_range = min_range;
	_max_range = max_range;
	_h_fov = h_fov;
	_v_fov = v_fov;


	device::Device::DeviceId device_id;

	device_id.devid_s.devtype = 0xF0;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/uart_configSx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	_px4_rangefinder.set_orientation(rotation);

	_px4_rangefinder.set_min_distance(min_range);
	_px4_rangefinder.set_max_distance(max_range);
	_px4_rangefinder.set_hfov(h_fov);
	_px4_rangefinder.set_vfov(v_fov);
}

Microbrain_H201_1::~Microbrain_H201_1() {
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int Microbrain_H201_1::init() {
	int ret = open_port();

	if (ret == PX4_OK) {
		start();
	}

	// close the fd
	::close(_fd);
	_fd = -1;

	return ret;
}

int
Microbrain_H201_1::open_port() {
    	int ret = PX4_OK;
    	do {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
		_fds.fd = _fd;
		_fds.events = POLLIN;


		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			PX4_ERR("port %s", _port);
			return -1;
		}

		unsigned speed = B115200;
		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd, &uart_config);

		// clear ONLCR flag (which appends a CR for every LF)
		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag = (uart_config.c_cflag & ~CSIZE) | CS8;
		uart_config.c_iflag &= ~IGNBRK;
		uart_config.c_lflag = 0;
		uart_config.c_oflag = 0;
		uart_config.c_cc[VMIN]  = 0;
		uart_config.c_cc[VTIME] = 5;
		uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);
		uart_config.c_cflag |= (CLOCAL | CREAD);
		uart_config.c_cflag &= ~(PARENB | PARODD);
		uart_config.c_cflag &= ~CSTOPB;
		uart_config.c_cflag &= ~CRTSCTS;

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			break;
		}


		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}
	} while (0);
	return ret;
}

void
Microbrain_H201_1::start()
{
	// schedule a cycle to start things (the sensor sends at 20Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(40_ms);
}

void
Microbrain_H201_1::stop()
{
	ScheduleClear();
}


void
Microbrain_H201_1::Run()
{
	// perform collection
	if (collect() == -EAGAIN) {
		// reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(40_ms, 40_ms);
		return;
	}
}


void
Microbrain_H201_1 ::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}


int Microbrain_H201_1::collect()
 {

    	// parse entire buffer
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	float distance_m = -1.0f;


	perf_begin(_sample_perf);

	// clear buffer if last read was too long ago
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	if (_fd < 0) {
		open_port();
	}

	if (_fd < 0) {
		PX4_ERR("Unable to open the port %s", _port);
		return PX4_ERROR;
	}

	int ret = 0;

    	do {
		uint8_t read_count = 0;
		uint8_t header_byte[1];
		ret = ::read(_fd, &header_byte, 1);

		if (ret < 0) {
			PX4_ERR("header_byte read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			// only throw an error if we time out
			if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
				/* flush anything in RX buffer */
				tcflush(_fd, TCIFLUSH);
				return ret;

			} else {
				return -EAGAIN;
			}
		}
		++read_count;

		if (header_byte[0] == 0x48) {
			uint8_t data_l[1];
			ret = ::read(_fd, &data_l, 1);
			if (ret < 0) {
				PX4_ERR("data_l read err: %d", ret);
				perf_count(_comms_errors);
				perf_end(_sample_perf);

				// only throw an error if we time out
				if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
					/* flush anything in RX buffer */
					tcflush(_fd, TCIFLUSH);
					return ret;
				} else {
					return -EAGAIN;
				}
			}
			++read_count;

			uint8_t data_h[1];
			ret = ::read(_fd, &data_h, 1);
			if (ret < 0) {
				PX4_ERR("data_h read err: %d", ret);
				perf_count(_comms_errors);
				perf_end(_sample_perf);

				// only throw an error if we time out
				if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
					/* flush anything in RX buffer */
					tcflush(_fd, TCIFLUSH);
					return ret;

				} else {
					return -EAGAIN;
				}
			}
			++read_count;

			if (data_h[0] != 255 && data_l[0] != 255) {
				int low = data_l[0] & 0x7F;
				int high = data_h[0] & 0x7F;
				distance_m = (high * 128) + low;
				distance_m = (distance_m * 2.5l) / 100;
				// publish most recent valid measurement from buffer
				if (distance_m > _min_range && distance_m < _max_range) {
					_px4_rangefinder.update(timestamp_sample, distance_m);
				}
				PX4_INFO("time: %llu distance: %0.2f", timestamp_sample, (double)distance_m);

			}

		}
	} while (0);

	if (distance_m < 0.0f) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}


	perf_end(_sample_perf);

	return PX4_OK;
}
