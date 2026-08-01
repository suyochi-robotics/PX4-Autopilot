/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/




/******************************************************************************************************** */
#include "Microbrain_R121_1.hpp"

#include <cerrno>
#include <mathlib/mathlib.h>

Microbrain_R121_1::Microbrain_R121_1(const char *module_name, const char *port, uint8_t rotation, float_t min_range,
				     float_t max_range,
				     float_t h_fov, float_t v_fov): ScheduledWorkItem(module_name, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation), _fd(-1)
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	_rotation = rotation;
	_min_range = min_range;
	_max_range = max_range;
	_h_fov = h_fov;
	_v_fov = v_fov;

	device::Device::DeviceId device_id{};

	device_id.devid_s.devtype = 0xF1;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	_px4_rangefinder.set_orientation(rotation);

	_px4_rangefinder.set_min_distance(min_range);
	_px4_rangefinder.set_max_distance(max_range);
	// The parameters are configured in degrees, while distance_sensor expects radians.
	_px4_rangefinder.set_hfov(math::radians(h_fov));
	_px4_rangefinder.set_vfov(math::radians(v_fov));
}

Microbrain_R121_1::~Microbrain_R121_1()
{
	stop();

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int Microbrain_R121_1::init()
{
	// File descriptors are local to the NuttX task that opens them. The driver
	// runs on a serial work queue, so defer opening the UART until Run().
	start();
	return PX4_OK;
}

int
Microbrain_R121_1::open_port()
{
	if (_fd >= 0) {
		return PX4_OK;
	}

	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("Error opening port %s", _port);
		return PX4_ERROR;
	}

	int ret = PX4_OK;
	termios uart_config{};
	int termios_state{};

	do {
		if (tcgetattr(_fd, &uart_config) < 0) {
			PX4_ERR("tcgetattr failed: %d", errno);
			ret = PX4_ERROR;
			break;
		}

		if ((termios_state = cfsetispeed(&uart_config, B115200)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = PX4_ERROR;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, B115200)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
			ret = PX4_ERROR;
			break;
		}

		uart_config.c_cflag = (uart_config.c_cflag & ~CSIZE) | CS8;
		uart_config.c_cflag |= (CLOCAL | CREAD);
		uart_config.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~(OPOST | ONLCR);

		uart_config.c_cc[VMIN] = 0;
		uart_config.c_cc[VTIME] = 0;

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = PX4_ERROR;
			break;
		}

		tcflush(_fd, TCIFLUSH);

	} while (0);

	if (ret != PX4_OK) {
		::close(_fd);
		_fd = -1;
	}

	return ret;
}

void
Microbrain_R121_1::start()
{
	// The sensor publishes at 20 Hz; poll slightly faster than the frame period.
	ScheduleOnInterval(kSampleInterval);
}

void
Microbrain_R121_1::stop()
{
	ScheduleClear();
}

void
Microbrain_R121_1::Run()
{
	collect();
}

void
Microbrain_R121_1::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

bool
Microbrain_R121_1::is_valid_distance(float distance_m) const
{
	return PX4_ISFINITE(distance_m) && distance_m >= _min_range && distance_m <= _max_range;
}

bool
Microbrain_R121_1::parse_frame(float &distance_m)
{
	if (_frame[kMarkerIndex] != kFrameMarker) {
		perf_count(_comms_errors);
		return false;
	}

	const uint8_t calculated_crc = calculateCRC(_frame, kCrcIndex);

	if (calculated_crc != _frame[kCrcIndex]) {
		perf_count(_comms_errors);
		return false;
	}

	const uint16_t distance_mm = (static_cast<uint16_t>(_frame[kDistanceMsbIndex]) << 8) | _frame[kDistanceLsbIndex];

	if (distance_mm == 0 || distance_mm == UINT16_MAX) {
		return false;
	}

	const float decoded_distance_m = static_cast<float>(distance_mm) * 0.001f;

	if (!is_valid_distance(decoded_distance_m)) {
		perf_count(_comms_errors);
		return false;
	}

	distance_m = decoded_distance_m;
	return true;
}

bool
Microbrain_R121_1::parse_byte(uint8_t byte, float &distance_m)
{
	switch (_parse_state) {
	case ParseState::WaitHeader1:
		if (byte == kFrameHeader1) {
			_frame[0] = byte;
			_parse_state = ParseState::WaitHeader2;
		}

		break;

	case ParseState::WaitHeader2:
		if (byte == kFrameHeader2) {
			_frame[1] = byte;
			_frame_index = 2;
			_parse_state = ParseState::ReadFrame;

		} else if (byte == kFrameHeader1) {
			_frame[0] = byte;

		} else {
			_parse_state = ParseState::WaitHeader1;
		}

		break;

	case ParseState::ReadFrame:
		_frame[_frame_index++] = byte;

		if (_frame_index >= kFrameSize) {
			_parse_state = ParseState::WaitHeader1;
			_frame_index = 0;
			return parse_frame(distance_m);
		}

		break;
	}

	return false;
}

int Microbrain_R121_1::collect()
{
	perf_begin(_sample_perf);

	if (open_port() != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	int bytes_available = 0;

	if (::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available) < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_ERR("FIONREAD failed: %d", errno);
			perf_count(_comms_errors);
		}

		bytes_available = kReadBufferSize;
	}

	if (bytes_available <= 0) {
		if (_last_read != 0 && hrt_elapsed_time(&_last_read) > kNoDataTimeout) {
			_parse_state = ParseState::WaitHeader1;
			_frame_index = 0;
		}

		perf_end(_sample_perf);
		return -EAGAIN;
	}

	uint8_t read_buffer[kReadBufferSize] {};
	float distance_m = -1.0f;
	bool valid_sample = false;
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	while (bytes_available > 0) {
		const size_t bytes_to_read = bytes_available > static_cast<int>(sizeof(read_buffer)) ? sizeof(
						     read_buffer) : bytes_available;
		const ssize_t bytes_read = ::read(_fd, read_buffer, bytes_to_read);

		if (bytes_read > 0) {
			_last_read = hrt_absolute_time();

			for (ssize_t i = 0; i < bytes_read; i++) {
				if (parse_byte(read_buffer[i], distance_m)) {
					valid_sample = true;
				}
			}

			bytes_available -= bytes_read;

		} else if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
			break;

		} else {
			PX4_ERR("read error: %d", errno);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			::close(_fd);
			_fd = -1;
			return PX4_ERROR;
		}
	}

	if (!valid_sample) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);

	return PX4_OK;
}

uint8_t Microbrain_R121_1::calculateCRC(const uint8_t data[], size_t length)
{
	uint8_t crc = 0x00;

	for (size_t i = 0; i < length; i++) {
		crc = crc8_table[crc ^ data[i]];
	}

	return crc;
}
