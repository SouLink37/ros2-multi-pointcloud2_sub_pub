# cmake_minimum_required(VERSION 3.5)
# project(pcd_subscriber)

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common io)
# # find_package(PCL 1.8 REQUIRED)

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# add_executable(pcd_subscriber
#   src/pcd_subscriber.cpp
# )
# ament_target_dependencies(pcd_subscriber 
#   rclcpp 
#   sensor_msgs
#   PCL
# )

# target_link_libraries(pcd_subscriber ${PCL_LIBRARIES})

# ament_package()

cmake_minimum_required(VERSION 3.5)
project(pcd_subscriber)

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
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
# find_package(libboost_system REQUIRED)


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


include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)




add_executable(pcd_subscriber_node 
        src/pcd_subscriber.cpp
        )

ament_target_dependencies(pcd_subscriber_node 
        rclcpp
        sensor_msgs 
        # pcl_ros
        pcl_conversions
        # PCL
        # ${PCL_LIBRARIES}
        
        )


target_link_libraries(pcd_subscriber_node  ${PCL_LIBRARIES})

install(TARGETS
  pcd_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
