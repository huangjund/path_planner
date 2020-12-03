cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)

add_custom_command(
  OUTPUT path.pb.cc path.pb.h
  COMMAND protoc
  ARGS --cpp_out . ${CMAKE_CURRENT_SOURCE_DIR}/path.proto -I${CMAKE_CURRENT_SOURCE_DIR}
  DEPENDS path.proto
)

