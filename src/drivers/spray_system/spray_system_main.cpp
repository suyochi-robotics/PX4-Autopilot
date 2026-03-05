#include "SPRAY_SYSTEM.hpp"

extern "C" __EXPORT int spray_system_main(int argc, char *argv[])
{
	if (argc < 2) {
		return SpraySystem::print_usage();
	}

	if (!strcmp(argv[1], "start")) {
		return SpraySystem::main(argc, argv);
	}

	if (!strcmp(argv[1], "stop")) {
		SpraySystem::stop();
		return PX4_OK;
	}

	if (!strcmp(argv[1], "status")) {
		return SpraySystem::custom_command(argc - 1, argv + 1);
	}

	return SpraySystem::print_usage("unknown command");
}


