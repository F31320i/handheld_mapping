cmake_minimum_required(VERSION 3.8)
project(vector_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# The back end instantiates PCL's templated algorithms (VoxelGrid, GICP,
# KdTreeFLANN, PCLBase, search::Search) on its own point type, which is not one
# of the types PCL precompiles into libpcl_*.so. With precompilation on those
# symbols simply do not exist and the link fails; this makes the template
# implementations header-only. Defined for the whole package so every
# translation unit sees the same PCL configuration.
add_compile_definitions(PCL_NO_PRECOMPILE)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree search registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# GTSAM was unpacked into this workspace's merge-install prefix (no sudo on this
# machine). build_sim.sh only sources /opt/ros/humble, so that prefix is not on
# CMAKE_PREFIX_PATH during the build -- add it explicitly before looking for the
# package. Harmless on a machine where GTSAM is installed system-wide.
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/lib/cmake")
find_package(GTSAM REQUIRED)

set(INCLUDE_DIRECTORIES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# Map accumulator: rebuilding fast_limo's map from the outside.
add_executable(map_accumulator_node src/map_accumulator_node.cpp)
target_include_directories(map_accumulator_node PUBLIC ${INCLUDE_DIRECTORIES})
target_compile_features(map_accumulator_node PUBLIC cxx_std_17)
target_link_libraries(map_accumulator_node ${PCL_LIBRARIES})

ament_target_dependencies(map_accumulator_node
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
)

# Pose-graph back end for FAST-LIVO2.
add_executable(pgo_backend_node src/pgo_backend_node.cpp)
target_include_directories(pgo_backend_node PUBLIC ${INCLUDE_DIRECTORIES})
target_compile_features(pgo_backend_node PUBLIC cxx_std_17)
target_link_libraries(pgo_backend_node ${PCL_LIBRARIES} gtsam)
set_target_properties(pgo_backend_node PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

ament_target_dependencies(pgo_backend_node
  Eigen3
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
  visualization_msgs
)

# Deskew check with synthetic motion: the simulator renders a whole sweep in a
# single tick, so this is the only place where the motion compensation is
# actually exercised.
add_executable(pgo_deskew_test src/pgo_deskew_test.cpp)
target_include_directories(pgo_deskew_test PUBLIC ${INCLUDE_DIRECTORIES})
target_compile_features(pgo_deskew_test PUBLIC cxx_std_17)
target_link_libraries(pgo_deskew_test ${PCL_LIBRARIES})

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  map_accumulator_node
  pgo_backend_node
  pgo_deskew_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_dependencies(
  PCL
  rclcpp
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following lines skip the linters for copyright and cpplint; comment them
  # out once a copyright header and a git repository are in place.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
