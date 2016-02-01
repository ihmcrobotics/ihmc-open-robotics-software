cmake_minimum_required(VERSION 2.6)

project(libpointmatcher)

#include(UseDoxygen)

# boost
#find_package(Boost REQUIRED)

# eigen 3
find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen
	/usr/include/eigen
)

# libpointmatcher
find_path(POINTMATCHER_INCLUDE_DIR pointmatcher/PointMatcher.h
  /usr/local/include
	/usr/include
)
find_library(POINTMATCHER_LIBRARY libpointmatcher
	/usr/local/lib
	/usr/lib
)


# nabo
find_path(NABO_INCLUDE_DIR nabo/nabo.h
	/usr/local/include
	/usr/include
)
find_library(NABO_LIBRARY libnabo
	/usr/local/lib
	/usr/lib
)

# look for OpenCL library
if (WIN32)
	find_library(OPENCL_LIBRARIES opencl64)
	if (!OPENCL_LIBRARIES)
		find_library(OPENCL_LIBRARIES opencl32)
	endif (!OPENCL_LIBRARIES)
else (WIN32)
	find_library(OPENCL_LIBRARIES OpenCL ENV LD_LIBRARY_PATH)
endif (WIN32)
# if found, add
if (OPENCL_LIBRARIES)
	set(EXTRA_LIBS ${OPENCL_LIBRARIES})
endif (OPENCL_LIBRARIES)

# include all libs so far
include_directories(${POINTMATCHER_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR} ${NABO_INCLUDE_DIR})
add_definitions(-Wall)

# test programs
add_executable(modular_cloud_matcher_batch ../modular_cloud_matcher_batch.cpp)
target_link_libraries(modular_cloud_matcher_batch ${POINTMATCHER_LIBRARY} ${NABO_LIBRARY} ${EXTRA_LIBS} rt)

