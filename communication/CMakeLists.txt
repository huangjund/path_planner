set(TOP_DIR ${CMAKE_CURRENT_LIST_DIR}/../)

find_package(Protobuf REQUIRED)
find_package(VnUtils REQUIRED)
find_package(OpenCV 3.0.0 REQUIRED)

if (NOT PROTOBUF_FOUND)
  message("===========Protobuf not found===========")
else()
  PROTOBUF_GENERATE_CPP(PROTOSRCS PROTOHDRS AGV.proto)
endif(NOT PROTOBUF_FOUND)

if (NOT VNUTILS_FOUND)
  message("===========VnUtils not found=========")
endif(NOT VNUTILS_FOUND)

if (NOT OPENCV_FOUND)
  message("===========Opencv not found===========")
endif(NOT OPENCV_FOUND)


include_directories(include ${VnUtils_INCLUDE_DIRS})
include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(ServerNode ccserver.cpp ccserver.h)
target_link_libraries(ServerNode ${PROTOBUF_LIBRARIES}
                                 ${VnUtils_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
                                 ${PROTOSRCS}
                                 ${PROTOHDRS})