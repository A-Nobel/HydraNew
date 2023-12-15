cmake_minimum_required(VERSION 3.1)
project(hydra_dsg_builder)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-Wno-error=register")
#zi ji wan
SET(Python2_INCLUDE_DIRS "/usr/include/python2.7")
SET(Python2_LIBRARIES "/usr/lib/python2.7/config-x86_64-linux-gnu/libpython2.7.so")
SET( PYTHON_EXECUTABLE /usr/bin/python2.7)
#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message("Python2: " ${Python2_INCLUDE_DIRS})
message("Python2: " ${Python2_LIBRARIES})



find_package(Boost REQUIRED COMPONENTS timer)
find_package(spark_dsg REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS hydra_topology
             kimera_pgmo
             kimera_semantics
             kimera_semantics_ros
             roscpp
             std_msgs
             tf2_eigen
             voxblox
             voxblox_ros
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(teaserpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(KimeraRPGO REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  hydra_topology
  kimera_pgmo
  kimera_semantics
  kimera_semantics_ros
  roscpp
  std_msgs
  tf2_eigen
  voxblox
  voxblox_ros
  DEPENDS spark_dsg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories( ${Python2_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS}  )
# include directories( ${Python2_INCLUDE_DIRS} )
# add_library(libhydra_dsg_builder src/incremental_dsg_frontend.cpp)
# target_link_libraries(libhydra_dsg_builder ${Python2_LIBRARIES})

add_library(
  ${PROJECT_NAME}
  src/dsg_lcd_descriptors.cpp
  src/dsg_lcd_matching.cpp
  src/dsg_lcd_detector.cpp
  src/dsg_lcd_registration.cpp
  src/dsg_update_functions.cpp
  src/incremental_dsg_backend.cpp
  src/incremental_dsg_frontend.cpp
  src/incremental_dsg_lcd.cpp
  src/incremental_mesh_segmenter.cpp
  src/incremental_room_finder.cpp
  src/lcd_visualizer.cpp
  src/minimum_spanning_tree.cpp
  src/visualizer_plugins.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
  teaserpp::teaser_registration
  spark_dsg::spark_dsg
)

# add_executable(testnode src/node.cpp)
# add_dependencies(testnode maintest_gencpp)
# target_link_libraries(testnode ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME}_incremental_node src/incremental_dsg_builder_node.cpp)
target_link_libraries(${PROJECT_NAME}_incremental_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${Python2_LIBRARIES})
add_dependencies(${PROJECT_NAME}_incremental_node ${PROJECT_NAME}_generate_messages_cpp)
#531 service
add_dependencies(${PROJECT_NAME}_incremental_node ${PROJECT_NAME}_gencpp)
add_executable(dsg_optimizer_node src/dsg_optimizer_node.cpp)
target_link_libraries(dsg_optimizer_node ${catkin_LIBRARIES} ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    utest_${PROJECT_NAME}
    tests/utest_main.cpp
    tests/utest_dsg_lcd_registration.cpp
    tests/utest_dsg_lcd_descriptors.cpp
    tests/utest_dsg_lcd_matching.cpp
    tests/utest_dsg_lcd_module.cpp
    tests/utest_dsg_update_functions.cpp
    tests/utest_incremental_room_finder.cpp
    tests/utest_minimum_spanning_tree.cpp
  )
  target_link_libraries(utest_${PROJECT_NAME} ${PROJECT_NAME})
endif()

# TODO(nathan) handle install
