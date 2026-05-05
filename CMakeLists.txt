cmake_minimum_required(VERSION 3.5)
project(bt_patrolling)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

execute_process(
  COMMAND /usr/bin/python3 -c "import ament_package"
  RESULT_VARIABLE AMENT_PACKAGE_IMPORT_RESULT
  OUTPUT_QUIET
  ERROR_QUIET)

if(NOT AMENT_PACKAGE_IMPORT_RESULT EQUAL 0)
  file(GLOB ROS_PYTHON_SITE_PACKAGES "/opt/ros/*/lib/python*/site-packages")
  foreach(ROS_SITE_PACKAGE_DIR ${ROS_PYTHON_SITE_PACKAGES})
    if(EXISTS "${ROS_SITE_PACKAGE_DIR}/ament_package")
      if(DEFINED ENV{PYTHONPATH} AND NOT "$ENV{PYTHONPATH}" STREQUAL "")
        set(ENV{PYTHONPATH} "${ROS_SITE_PACKAGE_DIR}:$ENV{PYTHONPATH}")
      else()
        set(ENV{PYTHONPATH} "${ROS_SITE_PACKAGE_DIR}")
      endif()
      break()
    endif()
  endforeach()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(nav2_msgs QUIET)
if(NOT nav2_msgs_FOUND)
  get_filename_component(WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
  set(NAV2_MSGS_PREFIX_HINTS
    "${WORKSPACE_ROOT}/install/nav2_msgs"
    "$ENV{NAV2_MSGS_PREFIX}"
  )

  find_package(nav2_msgs REQUIRED CONFIG
    PATHS ${NAV2_MSGS_PREFIX_HINTS}
    PATH_SUFFIXES share/nav2_msgs/cmake
    NO_DEFAULT_PATH)
endif()

find_package(behaviortree_cpp_v3 QUIET)
set(BTCPP_TARGET "")
set(BTCPP_INCLUDE_DIRS "")
set(BTCPP_LIBRARIES "")
if(behaviortree_cpp_v3_FOUND)
  if(TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
    set(BTCPP_TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
  elseif(TARGET behaviortree_cpp_v3)
    set(BTCPP_TARGET behaviortree_cpp_v3)
  else()
    set(BTCPP_INCLUDE_DIRS ${behaviortree_cpp_v3_INCLUDE_DIRS})
    set(BTCPP_LIBRARIES ${behaviortree_cpp_v3_LIBRARIES})
  endif()
  if(BTCPP_TARGET OR BTCPP_LIBRARIES)
    message(STATUS "Using system behaviortree_cpp_v3 package")
  else()
    message(WARNING "System behaviortree_cpp_v3 found but no usable targets/libs exported. Falling back to vendored build.")
  endif()
endif()

if(NOT BTCPP_TARGET AND NOT BTCPP_LIBRARIES)
  include(FetchContent)

  set(BTCPP_EXTRA_FLAGS "" CACHE STRING "")
  set(BTCPP_DISABLE_COROUTINES ON CACHE BOOL "" FORCE)
  set(BTCPP_SHARED_LIBS ON CACHE BOOL "" FORCE)
  set(BTCPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(BTCPP_BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)

  FetchContent_Declare(
    behaviortree_cpp_v3
    GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
    GIT_TAG 3.8.6
  )
  FetchContent_MakeAvailable(behaviortree_cpp_v3)

  if(TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
    set(BTCPP_TARGET behaviortree_cpp_v3::behaviortree_cpp_v3)
  elseif(TARGET behaviortree_cpp_v3)
    set(BTCPP_TARGET behaviortree_cpp_v3)
  else()
    message(FATAL_ERROR "Failed to resolve BehaviorTree.CPP target")
  endif()

  message(STATUS "Using vendored BehaviorTree.CPP via FetchContent")
endif()

set(BTCPP_LINK_LIBRARIES "")
if(BTCPP_TARGET)
  list(APPEND BTCPP_LINK_LIBRARIES ${BTCPP_TARGET})
elseif(BTCPP_LIBRARIES)
  list(APPEND BTCPP_LINK_LIBRARIES ${BTCPP_LIBRARIES})
else()
  message(FATAL_ERROR "Failed to resolve BehaviorTree.CPP link libraries")
endif()

find_package(ZMQ QUIET)
if(ZMQ_FOUND)
  message(STATUS "ZeroMQ found")
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies rclcpp rclcpp_action rclcpp_lifecycle action_msgs geometry_msgs lifecycle_msgs nav2_msgs ament_index_cpp)

include_directories(include ${ZMQ_INCLUDE_DIRS} ${BTCPP_INCLUDE_DIRS})

add_library(recharge_bt_node SHARED src/bt_patrolling/Recharge.cpp)
add_library(patrol_bt_node SHARED src/bt_patrolling/Patrol.cpp)
add_library(get_waypoint_bt_node SHARED src/bt_patrolling/GetWaypoint.cpp)
add_library(move_bt_node SHARED src/bt_patrolling/Move.cpp)
add_library(battery_checker_bt_node SHARED src/bt_patrolling/BatteryChecker.cpp)
add_library(track_objects_bt_node SHARED src/bt_patrolling/TrackObjects.cpp)

list(APPEND plugin_libs
  recharge_bt_node
  patrol_bt_node
  move_bt_node
  get_waypoint_bt_node
  battery_checker_bt_node
  track_objects_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_link_libraries(${bt_plugin} ${BTCPP_LINK_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(patrolling_main src/patrolling_main.cpp)
ament_target_dependencies(patrolling_main ${dependencies})
target_link_libraries(patrolling_main ${BTCPP_LINK_LIBRARIES})
if(ZMQ_FOUND)
  target_compile_definitions(patrolling_main PRIVATE ZMQ_FOUND)
  target_link_libraries(patrolling_main ${ZMQ_LIBRARIES})
endif()

install(TARGETS
  ${plugin_libs}
  patrolling_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()


