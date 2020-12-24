# SET( EMPTYHEADERS
#   ${CMAKE_CURRENT_SOURCE_DIR}/../common/hRScurve.h
# )

# SET(EMPTYSOURCES
#   ${CMAKE_CURRENT_SOURCE_DIR}/../common/hRScurve.cpp
# )

add_executable(empty empty.cpp ${HEADERS} ${SOURCES})
target_link_libraries(empty ${catkin_LIBRARIES})
target_link_libraries(empty ${OMPL_LIBRARIES})