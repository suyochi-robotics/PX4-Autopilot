message( STATUS ">>>>>>Building sensors__terrain_following<<<<<<<")

px4_add_module(
    MODULE sensors__terrain_following
	MAIN terrain_following
	SRCS
		TerrainFollowing.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		drivers_microbrain_h201_1
	)
