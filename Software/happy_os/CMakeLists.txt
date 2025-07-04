cmake_minimum_required(VERSION 3.16)
project(happy_ros2_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## CORROSION
include(FetchContent)

FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    GIT_TAG v0.4 # Optionally specify a commit hash, version tag or branch here
)
# Set any global configuration variables such as `Rust_TOOLCHAIN` before this line!
FetchContent_MakeAvailable(Corrosion)

# Import targets defined in a package or workspace manifest `Cargo.toml` file
corrosion_import_crate(MANIFEST_PATH hardware/motor_interface/Cargo.toml)

## COMPILE

add_library(
  happy_ros2_control
  SHARED
  hardware/happy_ros2_control/happy_system.cpp
)
target_compile_features(happy_ros2_control PUBLIC cxx_std_17)
target_include_directories(happy_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/happy_ros2_control/include>
$<INSTALL_INTERFACE:include/happy_ros2_control>
$<INSTALL_INTERFACE:include/motor_interface>
)
target_link_libraries(happy_ros2_control PRIVATE happy_hardware_interface)
ament_target_dependencies(
  happy_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "happy_ros2_control_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface happy_ros2_control.xml)

# INSTALL
install(
  DIRECTORY hardware/happy_ros2_control/include/
  DESTINATION include/happy_ros2_control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/happy_ros2_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/happy_ros2_control
)
install(TARGETS happy_ros2_control
  EXPORT export_happy_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_happy_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
