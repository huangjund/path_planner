cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)


## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    )

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/common/map/map.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/common/statespace/GridState.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/common/statespace/SE2State.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/Helper.cpp
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/common/map/map.h
    ${CMAKE_CURRENT_SOURCE_DIR}/common/statespace/GridState.h
    ${CMAKE_CURRENT_SOURCE_DIR}/common/statespace/State.h
    ${CMAKE_CURRENT_SOURCE_DIR}/common/statespace/SE2State.h
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/Helper.h
    )
add_library(HYAS ${SOURCES} ${HEADERS})

## Declare a catkin package
catkin_package()

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)
find_package(gflags REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

if(NOT GFLAGS_FOUND)
    message(AUTHOR_WARNING,"gflags not found")
endif(NOT GFLAGS_FOUND)


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})
include_directories(include ${GFLAGS_INCLUDE_DIRS})
include_directories(include ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include include)

# add_executable(tf_broadcaster examples/tf_broadcaster.cpp)
# target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(hybrid_astar examples/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(hybrid_astar ${catkin_LIBRARIES})
target_link_libraries(hybrid_astar ${OMPL_LIBRARIES})
target_link_libraries(hybrid_astar ${GFLAGS_LIBRARIES})

# install(TARGETS ${PROJECT_NAME} tf_broadcaster
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#     )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
