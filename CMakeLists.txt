cmake_minimum_required(VERSION 2.8.3)
project(asr_kinematic_chain_optimizer)

find_package(catkin REQUIRED COMPONENTS)

find_package(cmake_modules REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(TinyXML REQUIRED)

find_package(PkgConfig)
pkg_check_modules (MUPARSER REQUIRED muparser)

catkin_package(
  INCLUDE_DIRS include)
  
include_directories(include 
    ${catkin_INCLUDE_DIRS} 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${TinyXML_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${MUPARSER_INCLUDE_DIRS}
    )

add_executable(kinematic_chain_optimizer
    src/main.cpp
    src/configuration.cpp
    src/datapairs.cpp
    src/frame.cpp
    src/log.cpp
    src/optimizer.cpp
    src/optimizer_gnuplot.cpp
    src/optimizer_iv.cpp
    src/optimizer_ros.cpp
    src/utils.cpp
    src/vecmath.cpp
  )

target_link_libraries(kinematic_chain_optimizer
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${TinyXML_LIBRARIES}
    ${CMAKE_LIBRARIES}
    ${MUPARSER_LIBRARIES}
 )

install(TARGETS kinematic_chain_optimizer
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

