#include "TF02iCAN.hpp"


// Constructor
TF02iCAN::TF02iCAN(uint32_t can_id, uint8_t rotation)
: ScheduledWorkItem(TF02I_CAN_MODULE_NAME, px4::wq_configurations::hp_default),
  _px4_rangefinder(0, rotation)
{

    PX4_INFO("Loading Parameters");
    // Load parameters
    param_get(param_find("SENS_TF02IC_ID"),   &_can_id);
    param_get(param_find("SENS_TF02IC_BAUD"), &_baudrate);
    param_get(param_find("SENS_TF02IC_INT"),  &_interval_ms);
    param_get(param_find("SENS_TF02IC_MAX"),  &_max_range);
    param_get(param_find("SENS_TF02IC_MIN"),  &_min_range);
    PX4_INFO("Loaded Parameters");

    // Configure PX4 device ID
    device::Device::DeviceId devid{};
    devid.devid_s.devtype  = 0x03; // unique devtype for TF02-i CAN
    devid.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_UAVCAN;
    devid.devid_s.address  = static_cast<uint8_t>(_can_id & 0x7F);

    _px4_rangefinder.set_device_id(devid.devid);
    _px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
    _px4_rangefinder.set_orientation(rotation);
    _px4_rangefinder.set_min_distance(_min_range);
    _px4_rangefinder.set_max_distance(_max_range);
    _px4_rangefinder.set_hfov(_hfov);
    _px4_rangefinder.set_vfov(_vfov);
    PX4_INFO("Object Created");

}

// Destructor
TF02iCAN::~TF02iCAN()
{
    stop();
    perf_free(_sample_perf);
    perf_free(_comms_errors);
}

// Initialize driver
int TF02iCAN::init()
{
	const char *can_device = "/dev/can0"; // CAN device node
	_can_fd = open(can_device, O_RDWR);
	if (_can_fd < 0) {
		perror("Failed to open CAN device");
		return -1;
	}

	printf("CAN device opened: %s\n", can_device);

	start();
	PX4_INFO("tf02i_can initialised");
	return PX4_OK;
}


// Start periodic updates
void TF02iCAN::start()
{
    ScheduleOnInterval(_interval_ms * 1000); // µs
}

// Stop driver
void TF02iCAN::stop()
{
    ScheduleClear();

    if (_can_fd >= 0) {
        ::close(_can_fd);
        _can_fd = -1;
    }
}


// Main loop
void TF02iCAN::Run()
{
    perf_begin(_sample_perf);

    if (collect() != PX4_OK) {
        perf_count(_comms_errors);
    }

    perf_end(_sample_perf);
}

// Collect data from CAN sensor
int TF02iCAN::collect()
{
	struct can_msg_s frame;
	ssize_t n = read(_can_fd, &frame, sizeof(frame));
	if (n < 0) {
		perror("CAN read failed");
		return -1;
	}

	printf("Received CAN frame: ID=0x%X, LEN=%d, DATA=", frame.cm_hdr.ch_id, frame.cm_hdr.ch_len);
	for (int i = 0; i < frame.cm_hdr.ch_len; i++) {
	    printf("%02X ", frame.cm_data[i]);
	}
	printf("\n");
	return PX4_ERROR; // no valid frame
}

// Print driver info
void TF02iCAN::print_info()
{
    PX4_INFO("TF02-i CAN driver:");
    PX4_INFO("  CAN ID: %ld", _can_id);
    PX4_INFO("  Baudrate: %ld", _baudrate);
    PX4_INFO("  Interval: %ld ms", _interval_ms);
    PX4_INFO("  Min range: %.2f m", (double)_min_range);
    PX4_INFO("  Max range: %.2f m", (double)_max_range);

    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
}
