
add_library(TCPServer SHARED TCPServer.cc TCPConnection.cc  )
add_executable(ServerTest ServerTest.cc)
target_link_libraries(ServerTest ${Boost_LIBRARIES} pthread TCPServer)


add_library(DRCSimIHMCPlugin SHARED DRCSimIHMCPlugin.cpp)
target_link_libraries(DRCSimIHMCPlugin ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} TCPServer)