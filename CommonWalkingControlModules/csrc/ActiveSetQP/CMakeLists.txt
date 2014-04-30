# how http://www.bogotobogo.com/cplusplus/eclipse_CDT_JNI_MinGW_64bit.php
cmake_minimum_required(VERSION 2.8)
project(ActiveSetQP)
#set(POD_NAME ActiveSetQP)
#include(pods.cmake)

#pods_find_pkg_config(eigen3)
#if (eigen3_FOUND)
#	message("found eigen 3")
#	pods_use_pkg_config_includes( eigen3 )
#endif()

add_library(ActiveSetQP  QP.cpp)
set_target_properties(ActiveSetQP PROPERTIES COMPILE_FLAGS -fPIC)

if(MATLAB)
	include(mex.cmake)
	add_mex(fastQPmex fastQPmex.cpp)
	target_link_libraries(fastQPmex ActiveSetQP)
	set_target_properties(fastQPmex PROPERTIES COMPILE_FLAGS -fPIC)
	install(TARGETS fastQPmex RUNTIME DESTINATION lib LIBRARY DESTINATION lib)
	#pods_use_pkg_config_packages(fastQPmex eigen3)
endif(MATLAB)
