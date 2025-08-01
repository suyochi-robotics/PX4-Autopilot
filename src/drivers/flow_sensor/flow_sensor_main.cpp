
#include "FLOW_SENSOR.hpp"

 extern "C" __EXPORT int flow_sensor_main(int argc, char *argv[])
 {
	if (argc >= 2 && !strcmp(argv[1], "stop") && FlowSensor::is_running()) {
		FlowSensor::stop();
	}

	if (argc >= 2 && !strcmp(argv[1], "start") ) {
		return FlowSensor::main(argc, argv);
	}
	return PX4_OK;
 }
