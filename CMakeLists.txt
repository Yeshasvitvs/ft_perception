cmake_minimum_required(VERSION 2.6)
project(ft_perception)

find_package(YARP REQUIRED)
message(STATUS "YARP_INCLUDE_DIRS : " ${YARP_INCLUDE_DIRS})
message(STATUS "YARP_LIBRARIES : " ${YARP_LIBRARIES} )

find_package(ICUB REQUIRED)
message(STATUS "ICUB_INCLUDE_DIRS : " ${ICUB_INCLUDE_DIRS})
message(STATUS "ICUB_LIBRARIES : " ${ICUB_LIBRARIES})

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} -I include/)

add_executable(ft_perception src/main.cpp src/ft_module.cpp src/ft_estimate.cpp)

target_link_libraries(ft_perception ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

#install(TARGETS ft_perception RUNTIME DESTINATION bin)