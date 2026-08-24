cmake_minimum_required(VERSION 3.22)
project(coug_fgo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT _ipo_supported OUTPUT _ipo_error)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
if(_ipo_supported)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
else()
  message(STATUS "LTO not available, skipping: ${_ipo_error}")
endif()

# --- Dependencies ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(GTSAM REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED IMPORTED_TARGET geographiclib)

# --- GTSAM & Eigen Setup ---
get_target_property(GTSAM_TARGET_INC gtsam INTERFACE_INCLUDE_DIRECTORIES)
find_path(GTSAM_EIGEN_INCLUDE_DIR Eigen/Core
  PATHS ${GTSAM_TARGET_INC}
  PATH_SUFFIXES gtsam/3rdparty/Eigen
  NO_DEFAULT_PATH
)
if(NOT GTSAM_EIGEN_INCLUDE_DIR)
  message(FATAL_ERROR
    "Could not find GTSAM's vendored Eigen (Eigen/Core) under any of: "
    "${GTSAM_TARGET_INC}")
endif()

add_library(gtsam_with_eigen INTERFACE)
target_include_directories(gtsam_with_eigen SYSTEM INTERFACE
  "${GTSAM_EIGEN_INCLUDE_DIR}"
)
target_link_libraries(gtsam_with_eigen INTERFACE
  gtsam
  gtsam_unstable
)

# --- Nodes ---
set(COUG_FGO_NODES
  factor_graph
  dvl_a50_odom
  dvl_a50_twist_beams
  fluid_pressure_odom
  imu_ned_to_enu
  navsat_odom
  odom_ned_to_enu
  odom_to_tf
  sbg_imu_mag
  seatrac_x150_imu_depth
)

# --- Parameters ---
set(COUG_FGO_NODE_SOURCES "")
set(COUG_FGO_PARAM_LIBRARIES "")
foreach(node IN LISTS COUG_FGO_NODES)
  generate_parameter_library(
    ${node}_parameters
    "params/${node}_params.yaml"
  )
  list(APPEND COUG_FGO_NODE_SOURCES "src/${node}.cpp")
  list(APPEND COUG_FGO_PARAM_LIBRARIES "${node}_parameters")
endforeach()

# --- Component Library ---
ament_auto_add_library(factor_graph_component SHARED NO_TARGET_LINK_LIBRARIES
  src/factor_graph_core.cpp
  ${COUG_FGO_NODE_SOURCES}
)

target_compile_features(factor_graph_component PUBLIC cxx_std_17)

target_link_libraries(factor_graph_component
  gtsam_with_eigen
  ${COUG_FGO_PARAM_LIBRARIES}
  PkgConfig::GeographicLib
)

rclcpp_components_register_nodes(factor_graph_component
  "coug_fgo::FactorGraphNode"
  "coug_fgo::DvlA50OdomNode"
  "coug_fgo::DvlA50TwistBeamsNode"
  "coug_fgo::FluidPressureOdomNode"
  "coug_fgo::ImuNedToEnuNode"
  "coug_fgo::NavsatOdomNode"
  "coug_fgo::OdomNedToEnuNode"
  "coug_fgo::OdomToTfNode"
  "coug_fgo::SbgImuMagNode"
  "coug_fgo::SeatracX150ImuDepthNode"
)

# --- Executables ---
foreach(node IN LISTS COUG_FGO_NODES)
  ament_auto_add_executable(${node} NO_TARGET_LINK_LIBRARIES "src/${node}_node.cpp")
  target_link_libraries(${node} factor_graph_component)
endforeach()

# --- Python Bindings ---
pybind11_add_module(coug_fgo_py src/factor_graph_py.cpp)
target_link_libraries(coug_fgo_py PRIVATE
  factor_graph_component
)
install(FILES $<TARGET_FILE:coug_fgo_py>
  DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(COUG_FGO_FACTOR_TESTS
    test_ahrs_factor
    test_ahrs_origin_delta_factor
    test_ahrs_yaw_factor
    test_bearing_factor
    test_bearing_origin_delta_factor
    test_const_vel_factor
    test_depth_factor
    test_depth_origin_delta_factor
    test_dvl_factor
    test_dvl_loose_preint_factor
    test_dvl_tight_preint_factor
    test_gps_factor
    test_mag_calib_factor
    test_mag_factor
    test_range_factor
    test_range_origin_delta_factor
    test_wrench_dynamics_factor
  )

  foreach(test_name IN LISTS COUG_FGO_FACTOR_TESTS)
    ament_add_gtest(${test_name} "test/${test_name}.cpp")
    target_include_directories(${test_name} PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
    target_link_libraries(${test_name} gtsam_with_eigen)
  endforeach()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
