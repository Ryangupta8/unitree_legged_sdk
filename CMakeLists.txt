cmake_minimum_required(VERSION 3.8.3)
project(unitree_legged_sdk)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

## Ryan Added 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories("/usr/local/include/PnC")
include_directories(${PNC_INCLUDE_DIRS})

link_directories("/usr/local/lib")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(PnC REQUIRED)
## End Ryan Added.

include_directories(include)

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)
               # libA1PnC.so libmyFilters.so libmyMPC.so libmyRobotSystem.so
               # libmyPlanner.so libmyTrajectoryManager.so libmyUtils.so
               # libmyWBC.so libmyYaml.so)

set(EXTRA_LIBS ${EXTRA_LIBS} ${PNC_LIBRARIES} ${RYAN_TEST_LIBRARY})

set(CMAKE_CXX_FLAGS "-O3")

## Ryan exe
# add_executable(test_pnc src/test_pnc.cpp)
# target_link_libraries(test_pnc ${EXTRA_LIBS} ${PNC_LIBRARIES} ${RYAN_TEST_LIBRARY})

add_executable(test_pd src/test/test_pd.cpp)
target_link_libraries(test_pd ${EXTRA_LIBS} ${PNC_LIBRARIES} ${RYAN_TEST_LIBRARY})
## End Ryan exe


# one pc one process
add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity examples/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque examples/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk examples/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

# one pc multi process
add_executable(lcm_server_high examples/lcm_server_high.cpp)
target_link_libraries(lcm_server_high ${EXTRA_LIBS})

add_executable(lcm_server_low examples/lcm_server_low.cpp)
target_link_libraries(lcm_server_low ${EXTRA_LIBS})

# multi pc
add_executable(udp_send_test examples/multi_pc_udp_send.cpp)
target_link_libraries(udp_send_test ${EXTRA_LIBS})

add_executable(udp_recv_test examples/multi_pc_udp_recv.cpp)
target_link_libraries(udp_recv_test ${EXTRA_LIBS})