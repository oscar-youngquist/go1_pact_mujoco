cmake_minimum_required(VERSION 3.16)
project(go1_ros_wrapper)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(GO1_PACT_MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

set(UNITREE_SDK_DIR "$ENV{UNITREE_SDK_DIR}" CACHE PATH "Path to unitree_legged_sdk")
if(NOT UNITREE_SDK_DIR)
  set(UNITREE_SDK_DIR "${GO1_PACT_MUJOCO_DIR}/3rd_parties/unitree_legged_sdk")
endif()

set(MUJOCO_DIR "${GO1_PACT_MUJOCO_DIR}/DynamicsSim/Mujoco")
set(MUJOCO_BUILD_LIB_DIR "$ENV{MUJOCO_BUILD_LIB_DIR}" CACHE PATH "Directory containing libmujoco")
if(NOT MUJOCO_BUILD_LIB_DIR)
  set(MUJOCO_BUILD_LIB_DIR "${GO1_PACT_MUJOCO_DIR}/build/lib")
endif()

set(GO1_DEPLOY_DIR "$ENV{GO1_PACT_DEPLOY_DIR}" CACHE PATH "Path to go1_pact_deploy")
if(NOT GO1_DEPLOY_DIR)
  set(GO1_DEPLOY_DIR "${GO1_PACT_MUJOCO_DIR}/go1_pact_deploy")
endif()

set(TORCH_ROOT "$ENV{TORCH_ROOT}" CACHE PATH "LibTorch installation root")
if(NOT TORCH_ROOT)
  set(TORCH_ROOT "${GO1_PACT_MUJOCO_DIR}/3rd_parties/libtorch")
endif()

option(BUILD_UNITREE_BRIDGE "Build the Unitree hardware bridge if its SDK is available" ON)
option(BUILD_MUJOCO_SIM "Build the MuJoCo simulator if MuJoCo/GLFW are available" ON)
option(BUILD_PACT_CONTROLLER "Build the PACT controller if its dependencies are available" ON)

set(GO1_INSTALL_TARGETS)

add_executable(go1_pd_controller_node src/go1_pd_controller_node.cpp)
ament_target_dependencies(go1_pd_controller_node rclcpp sensor_msgs std_msgs)
list(APPEND GO1_INSTALL_TARGETS go1_pd_controller_node)

add_executable(go1_udp_gamepad_node src/go1_udp_gamepad_node.cpp)
ament_target_dependencies(go1_udp_gamepad_node rclcpp std_msgs)
list(APPEND GO1_INSTALL_TARGETS go1_udp_gamepad_node)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64.*")
  set(UNITREE_ARCH amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64.*")
  set(UNITREE_ARCH arm64)
else()
  set(UNITREE_ARCH unsupported)
endif()

set(UNITREE_SDK_INCLUDE_DIR "${UNITREE_SDK_DIR}/include")
find_library(
  UNITREE_SDK_LIB
  unitree_legged_sdk
  PATHS "${UNITREE_SDK_DIR}/lib/cpp/${UNITREE_ARCH}"
  NO_DEFAULT_PATH
)

if(BUILD_UNITREE_BRIDGE)
  if(EXISTS "${UNITREE_SDK_INCLUDE_DIR}" AND UNITREE_SDK_LIB)
    add_executable(go1_unitree_bridge_node src/go1_unitree_bridge_node.cpp)
    target_include_directories(go1_unitree_bridge_node PRIVATE "${UNITREE_SDK_INCLUDE_DIR}")
    target_link_libraries(go1_unitree_bridge_node ${UNITREE_SDK_LIB})
    ament_target_dependencies(go1_unitree_bridge_node rclcpp sensor_msgs std_msgs)
    list(APPEND GO1_INSTALL_TARGETS go1_unitree_bridge_node)
  else()
    message(WARNING "Skipping go1_unitree_bridge_node: UNITREE_SDK_DIR is not valid.")
  endif()
endif()

if(BUILD_MUJOCO_SIM)
  find_library(MUJOCO_LIB mujoco PATHS "${MUJOCO_BUILD_LIB_DIR}" NO_DEFAULT_PATH)
  find_path(
    GLFW_INCLUDE_DIR
    GLFW/glfw3.h
    PATHS
      "${GO1_PACT_MUJOCO_DIR}/build/_deps/glfw3-src/include"
      /usr/include
      /usr/local/include
  )
  find_library(GLFW_LIB NAMES glfw glfw3)

  if(MUJOCO_LIB AND EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h" AND GLFW_INCLUDE_DIR AND GLFW_LIB)
    find_package(OpenGL QUIET)
    find_package(X11 QUIET)

    if(OpenGL_FOUND AND X11_FOUND)
      add_executable(go1_mujoco_sim_node src/go1_mujoco_sim_node.cpp)
      target_include_directories(go1_mujoco_sim_node PRIVATE
        "${MUJOCO_DIR}/include"
        "${GLFW_INCLUDE_DIR}"
      )
      target_link_libraries(go1_mujoco_sim_node
        ${MUJOCO_LIB}
        ${GLFW_LIB}
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
      list(APPEND GO1_INSTALL_TARGETS go1_mujoco_sim_node)
    else()
      message(WARNING "Skipping go1_mujoco_sim_node: OpenGL or X11 was not found.")
    endif()
  else()
    message(WARNING "Skipping go1_mujoco_sim_node: MuJoCo or GLFW was not found.")
  endif()
endif()

if(BUILD_PACT_CONTROLLER)
  find_package(Torch QUIET PATHS "${TORCH_ROOT}")
  set(PARAM_HANDLER_DIR "${GO1_PACT_MUJOCO_DIR}/Utils/ParamHandler")

  if(Torch_FOUND
      AND EXISTS "${GO1_DEPLOY_DIR}/include/pact_controller.hpp"
      AND EXISTS "${UNITREE_SDK_INCLUDE_DIR}"
      AND EXISTS "${PARAM_HANDLER_DIR}/CMakeLists.txt")
    if(NOT TARGET daros_yaml-cpp)
      add_subdirectory(
        "${PARAM_HANDLER_DIR}"
        "${CMAKE_CURRENT_BINARY_DIR}/daros_yaml_cpp"
        EXCLUDE_FROM_ALL
      )
    endif()

    add_executable(go1_pact_controller_node src/go1_pact_controller_node.cpp)
    target_include_directories(go1_pact_controller_node PRIVATE
      "${GO1_DEPLOY_DIR}/include"
      "${GO1_DEPLOY_DIR}/src"
      "${GO1_PACT_MUJOCO_DIR}/3rd_parties"
      "${UNITREE_SDK_INCLUDE_DIR}"
      "${GO1_PACT_MUJOCO_DIR}/Utils"
      "${PARAM_HANDLER_DIR}"
    )
    target_link_libraries(go1_pact_controller_node ${TORCH_LIBRARIES} daros_yaml-cpp)
    set_target_properties(go1_pact_controller_node PROPERTIES
      BUILD_RPATH "${TORCH_ROOT}/lib"
      INSTALL_RPATH "${TORCH_ROOT}/lib"
    )
    ament_target_dependencies(go1_pact_controller_node rclcpp sensor_msgs std_msgs)
    list(APPEND GO1_INSTALL_TARGETS go1_pact_controller_node)
  else()
    message(WARNING "Skipping go1_pact_controller_node: Torch, go1_pact_deploy, Unitree headers, or ParamHandler are unavailable.")
  endif()
endif()

install(TARGETS ${GO1_INSTALL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/go1_training_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
