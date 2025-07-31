message(STATUS ">>>>>>>.Building lib Microbarin_H@01_1<<<<<<<<<<")
px4_add_library(
	drivers_microbrain_h201_1
	Microbrain_H201_1.cpp
	Microbrain_H201_1.hpp
	)

target_link_libraries(drivers_microbrain_h201_1
	PRIVATE
		drivers_rangefinder

)
