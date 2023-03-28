include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(test_io test_io.cpp)
target_compile_features(test_io PUBLIC cxx_std_11)
target_link_libraries(test_io promp gtest_main)

add_executable(test_learn test_learn.cpp)
target_compile_features(test_learn PUBLIC cxx_std_11)
target_link_libraries(test_learn promp gtest_main)

include(GoogleTest)
gtest_discover_tests(test_io WORKING_DIRECTORY ${CURRENT_SOURCE_DIR})
gtest_discover_tests(test_learn WORKING_DIRECTORY ${CURRENT_SOURCE_DIR})