add_subdirectory(ParamHandler)
file(GLOB sources "*.cpp")  
add_library(daros-utils SHARED ${sources})
target_link_libraries(daros-utils daros_param_handler)


