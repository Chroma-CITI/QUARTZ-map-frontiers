cmake_minimum_required(VERSION 2.8.3)
project(map_frontiers)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED COMPONENTS rospy)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

catkin_package()

add_definitions(-std=c++14 -Wall -Wextra)

# =========================
# Third-party: regularized-k-means (repo B)
# =========================
find_package(Threads REQUIRED)

# Adjust this if your folder name/path differs.
set(REG_KMEANS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../regularized-k-means)

add_library(regularized_k_means_lib
  ${REG_KMEANS_DIR}/src/k_means.cc
  ${REG_KMEANS_DIR}/src/lasso_k_means.cc
  ${REG_KMEANS_DIR}/src/network_simplex.cc
  ${REG_KMEANS_DIR}/src/regularized_k_means.cc
)

target_include_directories(regularized_k_means_lib PUBLIC
  ${REG_KMEANS_DIR}/include
  ${REG_KMEANS_DIR}/third_party
)

target_link_libraries(regularized_k_means_lib ${CMAKE_THREAD_LIBS_INIT})

# =========================
# Third-party: hungarian-algorithm-cpp (submodule)
# =========================
set(HUNGARIAN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/hungarian)

add_library(hungarian_lib
  ${HUNGARIAN_DIR}/Hungarian.cpp
)

target_include_directories(hungarian_lib PUBLIC
  ${HUNGARIAN_DIR}
)

# repo is C++11; your project is C++14 so OK
target_compile_features(hungarian_lib PUBLIC cxx_std_11)

# =========================
# LIBRARIES (your package)
# =========================
cs_add_library(${PROJECT_NAME}
  src/ViewGenerator.cpp
)

target_link_libraries(${PROJECT_NAME} hungarian_lib)

# =========================
# BINARIES
# =========================

cs_add_executable(NBV_Selector
  src/NBV_Selector.cpp
)
add_dependencies(NBV_Selector ${catkin_EXPORTED_TARGETS})
#target_link_libraries(NBV_Selector ${PROJECT_NAME} regularized_k_means_lib)
target_link_libraries(NBV_Selector
  ${PROJECT_NAME}
  regularized_k_means_lib
  #hungarian_lib
)


cs_add_executable(ExplorationComparator
  src/ExplorationComparator.cpp
)
target_link_libraries(ExplorationComparator ${PROJECT_NAME})

cs_add_executable(ExplorationTracker
  src/ExplorationTracker.cpp
)
target_link_libraries(ExplorationTracker ${PROJECT_NAME})

# =========================
# EXPORT / INSTALL
# =========================
cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)