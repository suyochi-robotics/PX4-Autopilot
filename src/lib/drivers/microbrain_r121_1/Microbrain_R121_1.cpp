#include "Microbrain_R121_1.hpp"

Microbrain_R121_1::Microbrain_R121_1(const char* module_name, const char* port, uint8_t rotation, float_t min_range, float_t max_range, float_t h_fov, float_t v_fov):ScheduledWorkItem(module_name, px4::serial_port_to_wq(port)),
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

	device_id.devid_s.devtype = 0xF1;
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

Microbrain_R121_1::~Microbrain_R121_1() {
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int Microbrain_R121_1::init() {
	int ret = open_port();

	if (ret == PX4_OK) {
		start();
	}

	//close the fd
	::close(_fd);
	_fd = -1;

	return ret;
}

int
Microbrain_R121_1::open_port() {
    	int ret = PX4_OK;
    	do {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
		// _fds.fd = _fd;
		// _fds.events = POLLIN;


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
		uart_config.c_cc[VTIME] = 1;
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
Microbrain_R121_1::start()
{
	// schedule a cycle to start things (the sensor sends at 20Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(40_ms);
}

void
Microbrain_R121_1::stop()
{
	ScheduleClear();
}


void
Microbrain_R121_1::Run()
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
Microbrain_R121_1::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}


int Microbrain_R121_1::collect()
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

	// tcflush(_fd, TCIFLUSH);  // clear stale data
	// // Use a 0 ms timeout for non-blocking check
	// int poll_ret = ::poll(&_fds, 1, 100);

	// if (poll_ret < 0 || !(_fds.revents & POLLIN)) {
	// 	PX4_INFO("Data is not available on serial port");
	// 	return PX4_ERROR;
	// }

		// Check the number of bytes available in the buffer
	// int bytes_available = 0;
	// ::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	// if (!bytes_available) {
	// 	perf_end(_sample_perf);
	// 	PX4_ERR("No bytes available");
	// 	return PX4_ERROR;
	// }


	int ret = 0;

    	do {
		uint8_t all_bytes[20];
		uint8_t first_byte;
		ssize_t n1 = ::read(_fd, &first_byte, 1);
		PX4_DEBUG("First byte %d", first_byte);
		if (n1 == 1 && first_byte == 0x54) {
			uint8_t second_byte;
			ssize_t n2 = read(_fd, &second_byte, 1);
			PX4_DEBUG("Second byte %d", second_byte);
			if (n2 == 1 && second_byte == 0x48) {
				uint8_t data_byte;

				ssize_t counter = 0;
				uint64_t start_time = hrt_absolute_time();
				while (counter < 18) {
				    ssize_t n3 = read(_fd, &data_byte, 1);
				    if (n3 == 1) {
				        all_bytes[counter + 2] = data_byte;
				        ++counter;
				    }
				    if (hrt_elapsed_time(&start_time) > 5_ms) {
				        PX4_DEBUG("Timeout while reading full frame");
				        return -EAGAIN;
				    }
				}

				all_bytes[0] = first_byte;
				all_bytes[1] = second_byte;
				// for (ssize_t i = 0; i < 20; ++i) {
				// 	PX4_DEBUG("byte %d is %d", i, all_bytes[i]);
				// }
				if (all_bytes[2] == 255 || all_bytes[3] == 255) {
					PX4_DEBUG("Invalid data bytes: %d, %d", all_bytes[2], all_bytes[3]);
					continue;
				}
				uint8_t calc_crc = calculateCRC(all_bytes, 19);
				uint8_t actual_crc = all_bytes[19];
				if (calc_crc == actual_crc) {
					uint16_t distance_mm = (all_bytes[2] << 8) | all_bytes[3];
					distance_m = distance_mm / 1000.0;
					PX4_INFO("Distance: %f" , (double)distance_m);
					if (distance_m > _min_range && distance_m < _max_range) {
						_px4_rangefinder.update(timestamp_sample, distance_m);
					}
				} else {
					PX4_DEBUG("CRC mismatch!");
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
			} else {
				PX4_DEBUG("Second byte headed didn't match");
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
		} else {
			PX4_DEBUG("First byte headed didn't match");
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
	} while (0);

	if (distance_m < 0.0f) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}


	perf_end(_sample_perf);

	return PX4_OK;
}


uint8_t Microbrain_R121_1::calculateCRC(const uint8_t data[], size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}
