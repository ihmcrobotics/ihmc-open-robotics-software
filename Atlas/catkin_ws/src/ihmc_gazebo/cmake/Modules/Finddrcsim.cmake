file(GLOB drcsim_DIRECTORY "/usr/share/drcsim-*")
 


if(NOT "${drcsim_DIRECTORY}" STREQUAL "")
	set(drcsim_FOUND "YES")
	message(STATUS "drcsim found: ${drcsim_DIRECTORY}")
	set(drcsim_INCLUDE_DIRS "${drcsim_DIRECTORY}/ros/atlas_msgs/msg_gen/cpp/include/")
else()
	if(drcsim_FIND_REQUIRED)
		message(FATAL_ERROR "Required library drcsim not found")
	endif()
	set(drcsim_FOUND "NO")
endif()
