
add_subdirectory(pybind11)

find_package(Eigen3 REQUIRED)
find_package(Python COMPONENTS Interpreter Development)

pybind11_add_module(pypromp
    pypromp.cpp
    bind_trajectory.cpp
    bind_trajectory_group.cpp
    bind_csv_reader.cpp
    bind_promp.cpp
    bind_serializer.cpp
)

target_link_libraries(pypromp PUBLIC Eigen3::Eigen promp)
set_target_properties(pypromp PROPERTIES OUTPUT_NAME "promp")

install(
    TARGETS pypromp 
    DESTINATION ${Python_SITELIB}/
)