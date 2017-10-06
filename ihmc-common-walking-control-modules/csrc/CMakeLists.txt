cmake_minimum_required(VERSION 2.6)
project(CommonWalkingControlModules)
if(WIN32)
	set(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} /MT")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
	set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /MT")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif(WIN32)


include_directories(${JNI_INCLUDE_DIRS}) 

add_library(CVXMomentumOptimizerWithGRFPenalizedSmoother SHARED CVXMomentumOptimizerWithGRFPenalizedSmootherNative.c CVXMomentumOptimizerWithGRFPenalizedSmoother/util.c CVXMomentumOptimizerWithGRFPenalizedSmoother/solver.c CVXMomentumOptimizerWithGRFPenalizedSmoother/matrix_support.c CVXMomentumOptimizerWithGRFPenalizedSmoother/ldl.c)
if(NOT WIN32)
	target_link_libraries(CVXMomentumOptimizerWithGRFPenalizedSmoother m)
	if(APPLE)
		set_target_properties(CVXMomentumOptimizerWithGRFPenalizedSmoother PROPERTIES SUFFIX .dylib)
	endif(APPLE)
endif()

if(UNIX)
	find_package(PkgConfig)
	pkg_check_modules(PC_EIGEN eigen3)
	include_directories(${PC_EIGEN_INCLUDE_DIRS})
endif()



add_library(ActiveSetQPMomentumOptimizer SHARED ActiveSetQPMomentumOptimizer.cpp ActiveSetQP/QP.cpp)


INSTALL(TARGETS CVXMomentumOptimizerWithGRFPenalizedSmoother ActiveSetQPMomentumOptimizer  RUNTIME DESTINATION lib LIBRARY DESTINATION lib)
