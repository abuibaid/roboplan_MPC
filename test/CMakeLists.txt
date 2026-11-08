find_package(GTest REQUIRED)
if(NOT TARGET roboplan_example_models::roboplan_example_models)
  find_package(roboplan_example_models REQUIRED)
endif()

add_executable(test_mpc test_mpc.cpp)
set_property(TARGET test_mpc PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_mpc roboplan_mpc roboplan_example_models::roboplan_example_models
  GTest::gtest_main)

add_executable(test_robot_transformation test_robot_transformation.cpp)
set_property(TARGET test_robot_transformation PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_robot_transformation roboplan_mpc
  roboplan_example_models::roboplan_example_models GTest::gtest_main)

# If ament_gtest is detected, make the tests visible in ROS 2.
find_package(ament_cmake_gtest QUIET)
if(ament_cmake_gtest_FOUND)
  ament_add_gtest_test(test_mpc)
  ament_add_gtest_test(test_robot_transformation)
else()
  include(GoogleTest)
  gtest_add_tests(TARGET test_mpc)
  gtest_add_tests(TARGET test_robot_transformation)
endif()
