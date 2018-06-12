cmake_minimum_required(VERSION 3.3)
project(igor2_cpp)

set (ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
set (HEBI_DIR ${ROOT_DIR}/hebi)

find_package(SDL2 REQUIRED)

set(HEBI_CPP_SOURCES
  ${HEBI_DIR}/src/command.cpp
  ${HEBI_DIR}/src/feedback.cpp
  ${HEBI_DIR}/src/group_command.cpp
  ${HEBI_DIR}/src/group.cpp
  ${HEBI_DIR}/src/group_feedback.cpp
  ${HEBI_DIR}/src/group_info.cpp
  ${HEBI_DIR}/src/info.cpp
  ${HEBI_DIR}/src/robot_model.cpp
  ${HEBI_DIR}/src/lookup.cpp
  ${HEBI_DIR}/src/mac_address.cpp
  ${HEBI_DIR}/src/trajectory.cpp
  ${HEBI_DIR}/src/log_file.cpp)


#set(CMAKE_CXX_FLAGS_DEBUG "-fsanitize=address ${CMAKE_CXX_FLAGS_DEBUG}")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fxray-instrument")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fxray-instrument -fxray-instruction-threshold=12")
set(HEBI_CPP_LINK_DIRECTORY ${HEBI_DIR}/lib/linux_x86_64)
set(HEBI_CPP_INCLUDE_DIRECTORIES ${HEBI_DIR}/include ${HEBI_DIR}/include/Eigen ${HEBI_DIR}/src)

# Packaging sources into object library allows reuse if you have multiple targets
add_library(hebi_cpp STATIC ${HEBI_CPP_SOURCES})

set(hebi_input_src
  ${ROOT_DIR}/util/input/event_handler.cpp
  ${ROOT_DIR}/util/input/joystick.cpp
  ${ROOT_DIR}/util/input/joystick_mapper.cpp)

set(igor2_driver_src
  ${ROOT_DIR}/kits/igor2/igor2_demo.cpp)

set(igor2_lib_src
  ${ROOT_DIR}/kits/igor2/components/arm.cpp
  ${ROOT_DIR}/kits/igor2/components/body.cpp
  ${ROOT_DIR}/kits/igor2/components/chassis.cpp
  ${ROOT_DIR}/kits/igor2/components/configuration.cpp
  ${ROOT_DIR}/kits/igor2/components/igor.cpp
  ${ROOT_DIR}/kits/igor2/components/joystick_interface.cpp
  ${ROOT_DIR}/kits/igor2/components/leg.cpp)

add_library(hebi-input STATIC ${hebi_input_src})
add_library(hebi-igor2 STATIC ${igor2_lib_src})
add_executable(igor2_demo ${igor2_driver_src})

target_include_directories(hebi_cpp PUBLIC
  ${HEBI_CPP_INCLUDE_DIRECTORIES})
target_include_directories(hebi-input PUBLIC ${ROOT_DIR}
  ${SDL2_INCLUDE_DIRS}
  ${HEBI_CPP_INCLUDE_DIRECTORIES})
target_include_directories(hebi-igor2 PUBLIC ${ROOT_DIR}
  ${ROOT_DIR}/kits/igor2 ${SDL2_INCLUDE_DIRS}
  ${HEBI_CPP_INCLUDE_DIRECTORIES})
target_include_directories(igor2_demo PUBLIC ${ROOT_DIR}
  ${ROOT_DIR}/kits/igor2 ${SDL2_INCLUDE_DIRS}
  ${HEBI_CPP_INCLUDE_DIRECTORIES})

set(HEBI_C_LIB ${HEBI_CPP_LINK_DIRECTORY}/${CMAKE_SHARED_LIBRARY_PREFIX}hebi${CMAKE_SHARED_LIBRARY_SUFFIX})

target_link_libraries(hebi-input PUBLIC hebi_cpp ${HEBI_C_LIB} ${SDL2_LIBRARIES})
target_link_libraries(hebi-igor2 PUBLIC hebi-input)
target_link_libraries(igor2_demo PUBLIC hebi-igor2 pthread)
