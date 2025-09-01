cmake_minimum_required(VERSION 3.16)
project(hlb_sssp VERSION 2.5.0 LANGUAGES CXX C)

option(HLB_BUILD_SHARED "Build shared library" ON)
option(HLB_BUILD_STATIC "Build static library" ON)
option(HLB_BUILD_PYTHON "Build Python extension with pybind11" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(hlb_sssp_obj OBJECT
    src/hlb_sssp.cpp
    src/hlb_c.cpp
)
target_include_directories(hlb_sssp_obj PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
if (MSVC)
  target_compile_options(hlb_sssp_obj PRIVATE /O2)
else()
  target_compile_options(hlb_sssp_obj PRIVATE -O3 -march=native -fno-exceptions -fno-rtti)
endif()

if (HLB_BUILD_SHARED)
  add_library(hlb_sssp SHARED $<TARGET_OBJECTS:hlb_sssp_obj>)
  set_target_properties(hlb_sssp PROPERTIES OUTPUT_NAME "hlb_sssp")
endif()

if (HLB_BUILD_STATIC)
  add_library(hlb_sssp_static STATIC $<TARGET_OBJECTS:hlb_sssp_obj>)
  set_target_properties(hlb_sssp_static PROPERTIES OUTPUT_NAME "hlb_sssp")
endif()

include(GNUInstallDirs)
install(TARGETS hlb_sssp hlb_sssp_static
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (HLB_BUILD_PYTHON)
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(hlb_py bindings/hlb_py.cpp)
  target_link_libraries(hlb_py PRIVATE hlb_sssp)
  target_include_directories(hlb_py PRIVATE include)
endif()
