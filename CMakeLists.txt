cmake_minimum_required(VERSION 3.16)
project(pfcr_rail_deploy CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# no FMA contraction / fast-math: keeps results bit-comparable with golden/
if(MSVC)
  add_compile_options(/fp:precise /W3 /utf-8)
else()
  add_compile_options(-ffp-contract=off -Wall -Wextra)
endif()

set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime package root (include/, lib/)")

add_library(pfcr_core src/pfcr_controller.cpp src/policy_builtin.cpp src/policy_onnx.cpp)
target_include_directories(pfcr_core PUBLIC include)

if(ONNXRUNTIME_ROOT)
  find_library(ONNXRUNTIME_LIB onnxruntime PATHS "${ONNXRUNTIME_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
  target_include_directories(pfcr_core PUBLIC "${ONNXRUNTIME_ROOT}/include")
  target_compile_definitions(pfcr_core PUBLIC PFCR_WITH_ONNXRUNTIME)
  target_link_libraries(pfcr_core PUBLIC "${ONNXRUNTIME_LIB}")
endif()

add_executable(rail_main app/rail_main.cpp)
target_link_libraries(rail_main PRIVATE pfcr_core)
if(WIN32)
  target_link_libraries(rail_main PRIVATE winmm)
endif()

add_executable(golden_test tests/golden_test.cpp)
target_link_libraries(golden_test PRIVATE pfcr_core)

if(ONNXRUNTIME_ROOT AND WIN32)
  file(GLOB ORT_DLLS "${ONNXRUNTIME_ROOT}/lib/*.dll")
  foreach(t rail_main golden_test)
    foreach(dll ${ORT_DLLS})
      add_custom_command(TARGET ${t} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${dll}" $<TARGET_FILE_DIR:${t}>)
    endforeach()
  endforeach()
endif()

enable_testing()
add_test(NAME golden_builtin COMMAND golden_test ${CMAKE_CURRENT_SOURCE_DIR})
if(ONNXRUNTIME_ROOT)
  add_test(NAME golden_onnx COMMAND golden_test ${CMAKE_CURRENT_SOURCE_DIR} --onnx)
endif()
