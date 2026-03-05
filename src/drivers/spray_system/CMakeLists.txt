message(STATUS ">>>>>>>Building spray system driver<<<<<<<<")

px4_add_module(
	MODULE driver__spray_system
	MAIN spray_system
	SRCS
		SPRAY_SYSTEM.cpp
		spray_system_main.cpp
	DEPENDS
		uORB
		px4_work_queue
	)
