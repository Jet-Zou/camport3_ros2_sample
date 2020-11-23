cmake_minimum_required(VERSION 3.5)
project(percipio_cam3)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(PCL REQUIRED)
if(NOT PCL_FOUND)
    message(FATAL_ERROR "PCL library not found")
else()
    message(STATUS "################################################################")
    find_package(OpenCV REQUIRED)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV library not found")
    else()
        message(STATUS "OpenCV version: ${OpenCV_VERSION}")
        message(STATUS "PLC version: ${PLC_VERSION}")
        include_directories(${PCL_INCLUDE_DIRS})
        include_directories(${OpenCV_INCLUDE_DIRS})

        add_executable(percipio_publisher src/percipio_publisher.cpp)
        target_link_libraries(percipio_publisher   ${catkin_LIBRARIES})
        target_link_libraries(percipio_publisher -ltycam -lboost_system ${OpenCV_LIBS} ${PCL_LIBS})
        ament_target_dependencies(percipio_publisher 
            rclcpp
            tf2_ros
            tf2_geometry_msgs
            tf2_eigen 
            geometry_msgs
            nav_msgs
            sensor_msgs 
            pcl_conversions
        )

        install(TARGETS
            percipio_publisher
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()
endif()
ament_package()
