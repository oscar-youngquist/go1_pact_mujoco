cmake_minimum_required(VERSION 3.16)
project(go1_ros_wrapper)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(X11 REQUIRED)

set(GO1_PACT_MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(UNITREE_SDK_DIR "${GO1_PACT_MUJOCO_DIR}/3rd_parties/unitree_legged_sdk")
set(MUJOCO_DIR "${GO1_PACT_MUJOCO_DIR}/DynamicsSim/Mujoco")
set(MUJOCO_BUILD_LIB_DIR "${GO1_PACT_MUJOCO_DIR}/build/lib")
set(GLFW_INCLUDE_DIR "${GO1_PACT_MUJOCO_DIR}/build/_deps/glfw3-src/include")
set(GO1_DEPLOY_DIR "${GO1_PACT_MUJOCO_DIR}/go1_pact_deploy")
set(PARAM_HANDLER_DIR "${GO1_PACT_MUJOCO_DIR}/Utils/ParamHandler")
set(CUDA_TOOLKIT_ROOT_DIR "${GO1_PACT_MUJOCO_DIR}/3rd_parties/cuda-12.8")
set(CUDAToolkit_ROOT "${CUDA_TOOLKIT_ROOT_DIR}")
set(CMAKE_CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")
set(CMAKE_PREFIX_PATH "${GO1_PACT_MUJOCO_DIR}/3rd_parties/libtorch;${CMAKE_PREFIX_PATH}")
find_package(Torch REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64.*")
  set(UNITREE_ARCH amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64.*")
  set(UNITREE_ARCH arm64)
else()
  message(FATAL_ERROR "Unsupported architecture for Unitree SDK: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_library(
  UNITREE_SDK_LIB
  unitree_legged_sdk
  PATHS "${UNITREE_SDK_DIR}/lib/cpp/${UNITREE_ARCH}"
  NO_DEFAULT_PATH
)

if(NOT UNITREE_SDK_LIB)
  message(FATAL_ERROR "Unitree SDK library not found under ${UNITREE_SDK_DIR}/lib/cpp/${UNITREE_ARCH}")
endif()

add_executable(go1_unitree_bridge_node src/go1_unitree_bridge_node.cpp)
target_include_directories(go1_unitree_bridge_node PRIVATE "${UNITREE_SDK_DIR}/include")
target_link_libraries(go1_unitree_bridge_node ${UNITREE_SDK_LIB})
ament_target_dependencies(go1_unitree_bridge_node rclcpp sensor_msgs std_msgs)

add_executable(go1_mujoco_sim_node src/go1_mujoco_sim_node.cpp)
target_include_directories(go1_mujoco_sim_node PRIVATE
  "${MUJOCO_DIR}/include"
  "${GLFW_INCLUDE_DIR}"
)
target_link_directories(go1_mujoco_sim_node PRIVATE "${MUJOCO_BUILD_LIB_DIR}")
target_link_libraries(go1_mujoco_sim_node
  mujoco
  glfw3
  OpenGL::GL
  ${X11_LIBRARIES}
  dl
  pthread
)
set_target_properties(go1_mujoco_sim_node PROPERTIES
  BUILD_RPATH "${MUJOCO_BUILD_LIB_DIR}"
  INSTALL_RPATH "${MUJOCO_BUILD_LIB_DIR}"
)
ament_target_dependencies(go1_mujoco_sim_node rclcpp sensor_msgs std_msgs)

add_executable(go1_pd_controller_node src/go1_pd_controller_node.cpp)
ament_target_dependencies(go1_pd_controller_node rclcpp sensor_msgs std_msgs)

add_executable(go1_udp_gamepad_node src/go1_udp_gamepad_node.cpp)
ament_target_dependencies(go1_udp_gamepad_node rclcpp std_msgs)

add_executable(go1_pact_controller_node src/go1_pact_controller_node.cpp)
target_include_directories(go1_pact_controller_node PRIVATE
  "${GO1_DEPLOY_DIR}/include"
  "${GO1_DEPLOY_DIR}/src"
  "${GO1_PACT_MUJOCO_DIR}/3rd_parties"
  "${UNITREE_SDK_DIR}/include"
  "${GO1_PACT_MUJOCO_DIR}/Utils"
  "${PARAM_HANDLER_DIR}"
)
target_link_directories(go1_pact_controller_node PRIVATE "${MUJOCO_BUILD_LIB_DIR}")
target_link_libraries(go1_pact_controller_node ${TORCH_LIBRARIES} daros_yaml-cpp)
set_target_properties(go1_pact_controller_node PROPERTIES
  BUILD_RPATH "${MUJOCO_BUILD_LIB_DIR};${GO1_PACT_MUJOCO_DIR}/3rd_parties/libtorch/lib;${GO1_PACT_MUJOCO_DIR}/3rd_parties/cuda-12.8/lib64"
  INSTALL_RPATH "${MUJOCO_BUILD_LIB_DIR};${GO1_PACT_MUJOCO_DIR}/3rd_parties/libtorch/lib;${GO1_PACT_MUJOCO_DIR}/3rd_parties/cuda-12.8/lib64"
)
ament_target_dependencies(go1_pact_controller_node rclcpp sensor_msgs std_msgs)
install(TARGETS go1_unitree_bridge_node go1_mujoco_sim_node go1_pd_controller_node go1_udp_gamepad_node go1_pact_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
