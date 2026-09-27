cmake_minimum_required(VERSION 2.8.3)
project(voxblox_map)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall -Wextra)

####################
# SET SOURCE FILES #
####################

set("${PROJECT_NAME}_SRCS"
  src/voxblox_map.cpp
)

#############
# LIBRARIES #
#############


# include_directories( #super unsure chatgpt hack
#   ${CATKIN_DEVEL_PREFIX}/include
#   ${CMAKE_CURRENT_SOURCE_DIR}/../planning
# )

cs_add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
)


##########
# EXPORT #
##########
cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
