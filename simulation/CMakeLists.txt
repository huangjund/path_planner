include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(interactionUI vision_mock.cpp)
target_link_libraries(interactionUI ${catkin_LIBRARIES})

add_executable(simple_tf simple_tf.cpp)
target_link_libraries(simple_tf ${catkin_LIBRARIES})