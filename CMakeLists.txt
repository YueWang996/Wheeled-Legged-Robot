# !!! WARNING !!! AUTO-GENERATED FILE, PLEASE DO NOT MODIFY IT AND USE
# https://docs.platformio.org/page/projectconf/section_env_build.html#build-flags
#
# If you need to override existing CMake configuration or add extra,
# please create `CMakeListsUser.txt` in the root of project.
# The `CMakeListsUser.txt` will not be overwritten by PlatformIO.

cmake_minimum_required(VERSION 3.13)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

project("FocX_MAIN_CONTROL_UNIT" C CXX)

include(CMakeListsPrivate.txt)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/CMakeListsUser.txt)
include(CMakeListsUser.txt)
endif()

add_custom_target(
    Production ALL
    COMMAND platformio -c clion run "$<$<NOT:$<CONFIG:All>>:-e${CMAKE_BUILD_TYPE}>"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
    Debug ALL
    COMMAND platformio -c clion debug "$<$<NOT:$<CONFIG:All>>:-e${CMAKE_BUILD_TYPE}>"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB MPU6050 src/MPU6050/*.cpp)
file(GLOB UTILITIES src/utilities/*.cpp)
file(GLOB SERVO src/ServoSDK/*.cpp)
file(GLOB SERVO_H src/ServoSDK/*.h)
file(GLOB SRC src/*.cpp)
file(GLOB SRC_H src/*.h)

add_executable(Z_DUMMY_TARGET ${SRC_LIST} ${MPU6050} ${SERVO} ${SERVO_H} ${SRC} ${SRC_H} src/FuzzyPID.cpp src/FuzzyPID.h src/WebPage.h)
