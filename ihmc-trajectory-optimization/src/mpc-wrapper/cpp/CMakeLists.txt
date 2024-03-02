cmake_minimum_required(VERSION 3.4)
project(mpc)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})

set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(Eigen3 REQUIRED)
find_package(hpipm-cpp REQUIRED)

include_directories(
    /usr/include/eigen3/
)

set(MPC_SOURCES

    external/MPCExternal.h
    external/MPCExternal.cpp
)

add_library(mpc-wrapper SHARED ${MPC_SOURCES})

target_link_libraries(mpc-wrapper
    hpipm-cpp::hpipm-cpp
)

target_include_directories(mpc-wrapper PUBLIC
     /usr/include/
     headers/
     launcher/
     external/
     /usr/include/eigen3/

    #  include eigen
    ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS mpc-wrapper
	EXPORT targets
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)
install(FILES ${MPC_SOURCES} DESTINATION "${INSTALL_INC_DIR}")


# ---------------------------------------------------------------------------------------------------

add_executable(MPCSolverTest launcher/MPCSolverTest.cpp)

target_link_libraries(MPCSolverTest PRIVATE
    mpc-wrapper
    hpipm-cpp::hpipm-cpp
)


target_include_directories(MPCSolverTest PUBLIC
    /usr/include/eigen3/
    /usr/include/
    launcher/
    headers/
)


