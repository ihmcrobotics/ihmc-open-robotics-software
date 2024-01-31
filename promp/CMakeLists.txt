# Authors: Ivan Bergonzani, Vishnu Radhakrishnan, Waldez Gomes, Luigi Penco
cmake_minimum_required(VERSION 3.11)

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)

set(PROJECTNAME promp)
project(${PROJECTNAME})

set(promp_VERSION 1.0.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

option(BUILD_PYTHON_MODULE "build python module" OFF)

#dependencies
find_package(Eigen3 REQUIRED)

set(SOURCE_FILES ${SOURCE_FILES}
	src/main/cpp/trajectory.cpp
	src/main/cpp/io/csv_reader.cpp
	src/main/cpp/io/serializer.cpp
	src/main/cpp/promp.cpp
	src/main/cpp/trajectory_group.cpp
)

# compile promp lib
add_library(${PROJECTNAME} SHARED ${SOURCE_FILES})

target_include_directories(${PROJECTNAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECTNAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECTNAME} PUBLIC Eigen3::Eigen)

# Install promp library
generate_export_header(${PROJECTNAME})

install(TARGETS ${PROJECTNAME}
	EXPORT targets
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

configure_package_config_file("cmake/promp.cmake.in"
	"prompConfig.cmake"
	INSTALL_DESTINATION "lib/cmake/promp"
)

install(EXPORT targets
	FILE prompTargets.cmake
	NAMESPACE promp::
	DESTINATION lib/cmake/promp
)

write_basic_package_version_file("${promp_BINARY_DIR}/prompConfigVersion.cmake"
	VERSION ${promp_VERSION}
	COMPATIBILITY AnyNewerVersion
)

install(FILES
	"${promp_BINARY_DIR}/prompConfig.cmake"
	"${promp_BINARY_DIR}/prompConfigVersion.cmake"
	DESTINATION
	lib/cmake/promp
)

# compile examples and generate documentation
add_subdirectory(examples)
add_subdirectory(doc)

# googletests
# enable_testing()
# add_subdirectory(test)

# create promp python module
if(${BUILD_PYTHON_MODULE})
	add_subdirectory(python)
endif()