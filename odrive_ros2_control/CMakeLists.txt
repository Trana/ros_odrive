cmake_minimum_required(VERSION 3.8)
project(odrive_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
include_directories(../odrive_base/include)

ament_auto_add_library(
  odrive_ros2_control_plugin SHARED
  ../odrive_base/src/epoll_event_loop.cpp
  ../odrive_base/src/socket_can.cpp
  src/odrive_hardware_interface.cpp
)

ament_target_dependencies(odrive_ros2_control_plugin
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

target_compile_features(odrive_ros2_control_plugin PRIVATE cxx_std_20)

install(TARGETS odrive_ros2_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_motor_health test/test_motor_health.cpp)
  target_compile_features(test_motor_health PRIVATE cxx_std_20)
  add_test(NAME motor_health COMMAND test_motor_health)
  add_executable(test_support_fallback test/test_support_fallback.cpp)
  target_compile_features(test_support_fallback PRIVATE cxx_std_20)
  add_test(NAME support_fallback COMMAND test_support_fallback)
  add_executable(test_batch_deadline_review test/test_batch_deadline_review.cpp)
  target_compile_features(test_batch_deadline_review PRIVATE cxx_std_20)
  add_test(NAME batch_deadline_review COMMAND test_batch_deadline_review)
  add_executable(test_coherent_batch test/test_coherent_batch.cpp)
  target_compile_features(test_coherent_batch PRIVATE cxx_std_20)
  add_test(NAME coherent_batch COMMAND test_coherent_batch)
  add_executable(test_pacing_experiment test/test_pacing_experiment.cpp)
  target_compile_features(test_pacing_experiment PRIVATE cxx_std_20)
  add_test(NAME pacing_experiment COMMAND test_pacing_experiment)
  add_executable(test_encoder_timing test/test_encoder_timing.cpp)
  target_compile_features(test_encoder_timing PRIVATE cxx_std_20)
  add_test(NAME encoder_timing COMMAND test_encoder_timing)
  add_executable(test_paced_commands test/test_paced_commands.cpp)
  target_compile_features(test_paced_commands PRIVATE cxx_std_20)
  add_test(NAME paced_commands COMMAND test_paced_commands)
  add_executable(test_hardware_timing test/test_hardware_timing.cpp)
  target_compile_features(test_hardware_timing PRIVATE cxx_std_20)
  add_test(NAME hardware_timing COMMAND test_hardware_timing)
  add_executable(test_verified_idle_shutdown test/test_verified_idle_shutdown.cpp)
  target_compile_features(test_verified_idle_shutdown PRIVATE cxx_std_20)
  add_test(NAME verified_idle_shutdown COMMAND test_verified_idle_shutdown)
  ament_auto_add_executable(test_activation_driver
    test/test_activation_driver.cpp ../odrive_base/src/epoll_event_loop.cpp)
  target_compile_features(test_activation_driver PRIVATE cxx_std_20)
  add_test(NAME activation_driver COMMAND test_activation_driver)
  add_test(NAME activation_driver_timing_off COMMAND test_activation_driver)
  set_tests_properties(activation_driver_timing_off PROPERTIES ENVIRONMENT "MARVIN_TRACING_MODE=off")
  ament_auto_add_executable(test_shutdown_driver
    test/test_shutdown_driver.cpp ../odrive_base/src/epoll_event_loop.cpp)
  target_compile_features(test_shutdown_driver PRIVATE cxx_std_20)
  add_test(NAME shutdown_driver COMMAND test_shutdown_driver)
  set_tests_properties(shutdown_driver PROPERTIES TIMEOUT 10)
  add_test(NAME shutdown_driver_timing_off COMMAND test_shutdown_driver)
  set_tests_properties(shutdown_driver_timing_off PROPERTIES
    TIMEOUT 10 ENVIRONMENT "MARVIN_TRACING_MODE=off")
  add_test(NAME paced_twelve_axis_driver COMMAND test_shutdown_driver)
  set_tests_properties(paced_twelve_axis_driver PROPERTIES
    TIMEOUT 10 ENVIRONMENT "ODRIVE_TEST_TWELVE_AXES=1;MARVIN_TRACING_MODE=metrics")
  add_test(NAME phase_locked_twelve_axis_driver COMMAND test_shutdown_driver)
  set_tests_properties(phase_locked_twelve_axis_driver PROPERTIES TIMEOUT 10
    ENVIRONMENT "ODRIVE_TEST_TWELVE_AXES=1;ODRIVE_TEST_PHASE_LOCKED=1;MARVIN_TRACING_MODE=metrics")
  add_test(NAME phase_locked_driver_timing_off COMMAND test_shutdown_driver)
  set_tests_properties(phase_locked_driver_timing_off PROPERTIES TIMEOUT 10
    ENVIRONMENT "ODRIVE_TEST_TWELVE_AXES=1;ODRIVE_TEST_PHASE_LOCKED=1;MARVIN_TRACING_MODE=off")
  add_test(NAME coherent_batch_driver COMMAND test_shutdown_driver)
  set_tests_properties(coherent_batch_driver PROPERTIES TIMEOUT 10
    ENVIRONMENT "ODRIVE_TEST_TWELVE_AXES=1;ODRIVE_TEST_BATCH=1;MARVIN_TRACING_MODE=metrics")
  add_test(NAME coherent_batch_driver_off COMMAND test_shutdown_driver)
  set_tests_properties(coherent_batch_driver_off PROPERTIES TIMEOUT 10
    ENVIRONMENT "ODRIVE_TEST_TWELVE_AXES=1;ODRIVE_TEST_BATCH=1;MARVIN_TRACING_MODE=off")
  add_test(NAME spaced_batch_driver COMMAND test_shutdown_driver)
  set_tests_properties(spaced_batch_driver PROPERTIES TIMEOUT 10
    ENVIRONMENT "ODRIVE_TEST_TWELVE_AXES=1;ODRIVE_TEST_BATCH=1;ODRIVE_TEST_BATCH_GAP=1;MARVIN_TRACING_MODE=metrics")
  add_test(NAME spaced_batch_driver_off COMMAND test_shutdown_driver)
  set_tests_properties(spaced_batch_driver_off PROPERTIES TIMEOUT 10
    ENVIRONMENT "ODRIVE_TEST_TWELVE_AXES=1;ODRIVE_TEST_BATCH=1;ODRIVE_TEST_BATCH_GAP=1;MARVIN_TRACING_MODE=off")
  add_test(NAME hold_failure_diagnostic COMMAND test_shutdown_driver)
  set_tests_properties(hold_failure_diagnostic PROPERTIES TIMEOUT 10
    ENVIRONMENT "ODRIVE_TEST_TWELVE_AXES=1;ODRIVE_TEST_BATCH=1;ODRIVE_TEST_HOLD=1;ODRIVE_TEST_HOLD_REJECT=1;MARVIN_TRACING_MODE=metrics")
  add_test(NAME hold_failure_diagnostic_off COMMAND test_shutdown_driver)
  set_tests_properties(hold_failure_diagnostic_off PROPERTIES TIMEOUT 10
    ENVIRONMENT "ODRIVE_TEST_TWELVE_AXES=1;ODRIVE_TEST_BATCH=1;ODRIVE_TEST_HOLD=1;ODRIVE_TEST_HOLD_REJECT=1;MARVIN_TRACING_MODE=off")
  add_test(NAME support_hold_driver COMMAND test_shutdown_driver)
  set_tests_properties(support_hold_driver PROPERTIES TIMEOUT 10
    ENVIRONMENT "ODRIVE_TEST_TWELVE_AXES=1;ODRIVE_TEST_BATCH=1;ODRIVE_TEST_HOLD=1;MARVIN_TRACING_MODE=metrics")
endif()

ament_package()
