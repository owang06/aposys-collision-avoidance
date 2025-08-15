cmake_minimum_required(VERSION 3.25)
project(collision LANGUAGES CXX)

# ----- build options -----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ----- dependencies -----
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# ----- modules -----
file(GLOB SRC_FILES CONFIGURE_DEPENDS src/*.cpp)
pybind11_add_module(collision_geom ${SRC_FILES})

# ----- include paths -----
target_include_directories(collision_geom
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ----- link libraries -----
target_link_libraries(collision_geom
    PRIVATE
        Eigen3::Eigen
        pybind11::module
        $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>)

# ----- compiler flags -----
if(OpenMP_CXX_FOUND)
    target_compile_options(collision_geom PRIVATE ${OpenMP_CXX_FLAGS})
endif()

if(NOT MSVC)
    target_compile_options(collision_geom PRIVATE -O3 -ffast-math)
else()
    # Enable OpenMP on MSVC
    target_compile_options(collision_geom PRIVATE /openmp)
endif()

# ----- installation -----
# Also install headers for any C++ consumers
if(NOT DEFINED PYTHON_INSTALL_DIR)
  set(PYTHON_INSTALL_DIR ${Python3_SITEARCH})
endif()

install(TARGETS collision_geom
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
  RUNTIME DESTINATION ${PYTHON_INSTALL_DIR})

# Also install headers for potential C++ consumers
install(DIRECTORY include/ DESTINATION include)

message(STATUS "Python extensions will be installed to ${PYTHON_INSTALL_DIR}")


