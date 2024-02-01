cmake_minimum_required(VERSION 3.4)
project(slam-wrapper C CXX)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})

set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for headers")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(GTSAM REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})

set(SW_SOURCES
    cpp/factorGraph/FactorGraphHandler.cpp
    cpp/factorGraph/FactorGraphHandler.h
    cpp/external/FactorGraphExternal.cpp
    cpp/external/FactorGraphExternal.h
)

add_library(slam-wrapper SHARED ${SW_SOURCES})

target_link_libraries(slam-wrapper
    gtsam
)

target_include_directories(slam-wrapper PUBLIC
    cpp/factorGraph/
    cpp/external/
)

install(TARGETS slam-wrapper
	EXPORT targets
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)
install(FILES ${SW_SOURCES} DESTINATION "${INSTALL_INC_DIR}")

# +++++++++++++++++++++++++++++++++++++++ C++ Tester +++++++++++++++++++++++++++++++++++++++++++++++++++++++

add_executable(FactorGraphHeadless
    ${SW_SOURCES}
    cpp/launcher/Application.cpp
)

target_link_libraries(FactorGraphHeadless gtsam)

target_include_directories(FactorGraphHeadless PUBLIC
    cpp/factorGraph/
    cpp/external/
)
