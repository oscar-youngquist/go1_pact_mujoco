cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(PACT-Mujoco LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Your own custom cmake modules, if any
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Point CMake at the LibTorch install root
list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/3rd_parties/libtorch")

# Optional: if auto-discovery still fails, use this instead:
set(Torch_DIR "${PROJECT_SOURCE_DIR}/3rd_parties/libtorch/share/cmake/Torch")

set(CUDA_TOOLKIT_ROOT_DIR "${PROJECT_SOURCE_DIR}/3rd_parties/cuda-12.8")
set(CUDAToolkit_ROOT "${CUDA_TOOLKIT_ROOT_DIR}")
set(CMAKE_CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")

find_package(Torch REQUIRED)

set(THIS_COM "${PROJECT_SOURCE_DIR}/")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -no-pie -march=native -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs \
    -Wsign-promo -Wno-aligned-new -Wno-unused-but-set-variable \
    -Wno-type-limits -Wno-missing-field-initializers -Wno-unused-parameter")


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ggdb -march=native -I.")

if(EXISTS "/usr/include/eigen3")
    include_directories("/usr/include/eigen3")
else()
    include_directories("/usr/local/include/eigen3")
endif()
include_directories(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR}/Utils)
include_directories(${CMAKE_SOURCE_DIR}/Utils/ParamHandler)
include_directories(${CMAKE_SOURCE_DIR}/DynamicsSim/Mujoco/include)
include_directories(${CMAKE_SOURCE_DIR}/DynamicsSim/Mujoco/src)
include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(DynamicsSim/Mujoco)
add_subdirectory(Utils)
add_subdirectory(go1_pact_deploy)
