message( STATUS "building sensors__bw_obstacle_avoidance")

px4_add_module(
    MODULE sensors__bw_obstacle_avoidance
	MAIN bw_obstacle_avoidance
	SRCS
		BwObstacleAvoidance.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		drivers_microbrain_r121_1
	)
