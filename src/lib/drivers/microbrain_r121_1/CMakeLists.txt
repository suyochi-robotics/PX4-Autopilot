px4_add_library(
	drivers_microbrain_r121_1
	Microbrain_R121_1.hpp
	Microbrain_R121_1.cpp
	)

target_link_libraries(drivers_microbrain_r121_1
	PRIVATE
		drivers_rangefinder

)
