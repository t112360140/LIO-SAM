cmake_minimum_required(VERSION 3.5)
project(lio_sam)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(ROS_DISTRO $ENV{ROS_DISTRO})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)

find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)

find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs)

option(ENABLE_GPU "build with GPU" OFF)

include(CheckLanguage)
check_language(CUDA)
if(ENABLE_GPU AND CMAKE_CUDA_COMPILER)
  enable_language(CXX CUDA)
  add_definitions(-DHAS_CUDA)
  message(STATUS "CUDA detected. GPU Colorization enabled.")

  include_directories(
    /usr/local/cuda/include
  )
  link_directories(
    /usr/local/cuda/lib64
  )

  add_library(point_colorizer_cuda STATIC src/point_colorizer/point_colorizer.cu)
  set_target_properties(point_colorizer_cuda PROPERTIES
    CUDA_STANDARD 14
    CUDA_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
  )
else()
  message(WARNING "CUDA not detected. Falling back to CPU colorization.")
endif()

if(ROS_DISTRO STREQUAL "galactic")
	add_definitions(-DROS_GALACTIC)
		
	add_compile_options(-Wno-deprecated-declarations)

	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})

  add_executable(${PROJECT_NAME}_featureExtraction src/lio_sam/featureExtraction.cpp)
  ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
  rosidl_target_interfaces(${PROJECT_NAME}_featureExtraction ${PROJECT_NAME} "rosidl_typesupport_cpp")

  add_executable(${PROJECT_NAME}_imageProjection src/lio_sam/imageProjection.cpp)
  ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
  rosidl_target_interfaces(${PROJECT_NAME}_imageProjection ${PROJECT_NAME} "rosidl_typesupport_cpp")

  add_executable(${PROJECT_NAME}_imuPreintegration src/lio_sam/imuPreintegration.cpp)
  ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM Eigen)
  target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam)
  rosidl_target_interfaces(${PROJECT_NAME}_imuPreintegration ${PROJECT_NAME} "rosidl_typesupport_cpp")

  add_executable(${PROJECT_NAME}_mapOptimization src/lio_sam/mapOptmization.cpp src/scan_context/Scancontext.cpp)
  ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM cv_bridge)
  if (OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
    rosidl_target_interfaces(${PROJECT_NAME}_mapOptimization ${PROJECT_NAME} "rosidl_typesupport_cpp")
  else()
    target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam ${PCL_LIBRARIES})
    rosidl_target_interfaces(${PROJECT_NAME}_mapOptimization ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()

  add_executable(${PROJECT_NAME}_offline_runner
    src/offline_runner.cpp
    src/scan_context/Scancontext.cpp
    src/imu_tools/complementary_filter.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_offline_runner PRIVATE OFFLINE_MODE PROJECT_NAME="${PROJECT_NAME}")
  ament_target_dependencies(${PROJECT_NAME}_offline_runner rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM rosbag2_cpp rosbag2_storage cv_bridge image_transport image_geometry ament_index_cpp)
  if (OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_offline_runner gtsam ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
    rosidl_target_interfaces(${PROJECT_NAME}_offline_runner ${PROJECT_NAME} "rosidl_typesupport_cpp")
  else()
    target_link_libraries(${PROJECT_NAME}_offline_runner gtsam ${PCL_LIBRARIES})
    rosidl_target_interfaces(${PROJECT_NAME}_offline_runner ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()
  if(ENABLE_GPU AND CMAKE_CUDA_COMPILER)
    target_link_libraries(${PROJECT_NAME}_offline_runner point_colorizer_cuda cudart)
  endif()


  add_executable(${PROJECT_NAME}_mapViewerServer src/map_viewer_server.cpp)
  target_compile_definitions(${PROJECT_NAME}_mapViewerServer PRIVATE PROJECT_NAME="${PROJECT_NAME}")
  ament_target_dependencies(${PROJECT_NAME}_mapViewerServer rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL ament_index_cpp)
  rosidl_target_interfaces(${PROJECT_NAME}_mapViewerServer ${PROJECT_NAME} "rosidl_typesupport_cpp")

elseif(ROS_DISTRO STREQUAL "humble")
	add_definitions(-DROS_HUMBLE)

  add_executable(${PROJECT_NAME}_featureExtraction src/lio_sam/featureExtraction.cpp)
  ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${PROJECT_NAME}_featureExtraction "${cpp_typesupport_target}") 

  add_executable(${PROJECT_NAME}_imageProjection src/lio_sam/imageProjection.cpp)
  ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
  target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}") 

  add_executable(${PROJECT_NAME}_imuPreintegration src/lio_sam/imuPreintegration.cpp)
  ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM Eigen)
  target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam "${cpp_typesupport_target}")

  add_executable(${PROJECT_NAME}_mapOptimization src/lio_sam/mapOptmization.cpp src/scan_context/Scancontext.cpp)
  ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM cv_bridge)
  if (OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}" OpenMP::OpenMP_CXX)
  else()
    target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}")
  endif()

  add_executable(${PROJECT_NAME}_offline_runner
    src/offline_runner.cpp
    src/scan_context/Scancontext.cpp
    src/imu_tools/complementary_filter.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_offline_runner PRIVATE OFFLINE_MODE PROJECT_NAME="${PROJECT_NAME}")
  ament_target_dependencies(${PROJECT_NAME}_offline_runner rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM rosbag2_cpp rosbag2_storage cv_bridge image_transport image_geometry ament_index_cpp)
  if (OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME}_offline_runner gtsam "${cpp_typesupport_target}" OpenMP::OpenMP_CXX)
  else()
    target_link_libraries(${PROJECT_NAME}_offline_runner gtsam "${cpp_typesupport_target}")
  endif()
  if(ENABLE_GPU AND CMAKE_CUDA_COMPILER)
    target_link_libraries(${PROJECT_NAME}_offline_runner point_colorizer_cuda cudart)
  endif()


  add_executable(${PROJECT_NAME}_mapViewerServer src/map_viewer_server.cpp)
  target_compile_definitions(${PROJECT_NAME}_mapViewerServer PRIVATE PROJECT_NAME="${PROJECT_NAME}")
  ament_target_dependencies(${PROJECT_NAME}_mapViewerServer rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL ament_index_cpp)
  target_link_libraries(${PROJECT_NAME}_mapViewerServer "${cpp_typesupport_target}")

else()
    message(WARNING "Unknown ROS 2 Distribution: ${ROS_DISTRO}")
endif()



install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY web
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}_imageProjection
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_imuPreintegration
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_featureExtraction
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_mapOptimization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_offline_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_mapViewerServer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

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

ament_package()

