
message( STATUS "building sensors__fw_obstacle_avoidance")
px4_add_module(
    MODULE sensors__fw_obstacle_avoidance
	MAIN fw_obstacle_avoidance
	SRCS
		FwObstacleAvoidance.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		drivers_microbrain_r121_1
	)
