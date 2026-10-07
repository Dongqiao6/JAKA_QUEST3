cmake_minimum_required(VERSION 3.8)
project(jaka_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# This consolidated workspace only needs ``rc_ctrl_node`` for HTS -> JAKA
# teleoperation. The full ``jaka_driver`` node is intentionally not built here;
# its source remains available under ``src/jaka_driver.cpp`` if needed.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)

set(SDK_LIB "${CMAKE_CURRENT_SOURCE_DIR}/sdk_lib/libjakaAPI.so")

if(EXISTS ${SDK_LIB})
  add_executable(rc_ctrl_node
    src/rc_ctrl_node.cpp
    src/pgi140_gripper.cpp
    src/pgi140_protocol.cpp
  )

  ament_target_dependencies(rc_ctrl_node
    rclcpp
    geometry_msgs
    sensor_msgs
    jaka_msgs
  )

  target_include_directories(rc_ctrl_node
    PUBLIC
      include
    PRIVATE
      ${ament_INCLUDE_DIRS}
  )

  target_link_libraries(rc_ctrl_node ${SDK_LIB})

  install(TARGETS rc_ctrl_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(FILES ${SDK_LIB}
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "JAKA SDK not found at ${SDK_LIB}. "
    "rc_ctrl_node will NOT be built. Copy libjakaAPI.so into "
    "${CMAKE_CURRENT_SOURCE_DIR}/sdk_lib/ and rebuild."
  )
endif()

# Install launch files even when the SDK is absent so the package can still be
# found and users get a clear launch-time error instead of "package not found".
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES README_PGI140.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(pgi140_protocol_test
    test/pgi140_protocol_test.cpp
    src/pgi140_protocol.cpp
  )
  target_include_directories(pgi140_protocol_test PRIVATE include)
  add_test(NAME pgi140_protocol_test COMMAND pgi140_protocol_test)
endif()

ament_package()
