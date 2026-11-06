cmake_minimum_required(VERSION 3.22)
project(roboplan_mpc VERSION 0.8.0)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT
                                             STREQUAL "MSVC")
  # clang-cl remaps plain -Wall to MSVC /Wall (clang's -Weverything); use the
  # /clang: prefix to pass the GNU-style flags through unchanged.
  add_compile_options(/clang:-Wall /clang:-Wextra /clang:-Wpedantic)
elseif(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Windows: auto-export all symbols so DLLs get import libraries.
if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# Find dependencies.
if(NOT TARGET roboplan::roboplan)
  find_package(roboplan_core REQUIRED)
endif()

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite
# recursion issues with symlink installs.
find_package(ament_cmake QUIET)

# CasADi formulates and solves the optimal control problem; Pinocchio provides
# the robot dynamics on CasADi symbols (both come with pinocchio).
if(NOT TARGET pinocchio::pinocchio_casadi)
  find_package(pinocchio REQUIRED)
endif()

# Add libraries
add_library(roboplan_mpc SHARED src/mpc.cpp)
target_include_directories(
  roboplan_mpc PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)
target_link_libraries(roboplan_mpc PUBLIC roboplan::roboplan
                                          pinocchio::pinocchio_casadi)
set_property(TARGET roboplan_mpc PROPERTY CXX_STANDARD 20)
# Alias so this works via add_subdirectory() too, not just installed
# find_package().
add_library(roboplan_mpc::roboplan_mpc ALIAS roboplan_mpc)

install(
  TARGETS roboplan_mpc
  EXPORT roboplan_mpc-targets
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file("cmake/roboplan_mpcConfig.cmake.in"
               "${CMAKE_CURRENT_BINARY_DIR}/roboplan_mpcConfig.cmake" @ONLY)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/roboplan_mpcConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_mpcConfig.cmake
  INSTALL_DESTINATION lib/cmake/roboplan_mpc)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_mpcConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/roboplan_mpcConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/roboplan_mpcConfigVersion.cmake"
        DESTINATION "lib/cmake/roboplan_mpc")

# Install the headers and targets
install(DIRECTORY include/roboplan_mpc DESTINATION lib/cmake/)

install(
  EXPORT roboplan_mpc-targets
  FILE roboplan_mpcTargets.cmake
  NAMESPACE roboplan_mpc::
  DESTINATION lib/cmake/roboplan_mpc)

install(
  TARGETS roboplan_mpc
  EXPORT roboplan_mpc
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

# Build the Python bindings by default (for the cmeel wheel, colcon, and pixi).
# Override with -DBUILD_PYTHON_BINDINGS=OFF for a pure C++ build.
option(BUILD_PYTHON_BINDINGS "Build the roboplan_mpc Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

# If ament_cmake is detected, export the package to be visible in ROS 2.
if(ament_cmake_FOUND)
  message(STATUS "ament_cmake found. Exporting for ROS 2.")
  ament_package()
endif()
