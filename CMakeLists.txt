cmake_minimum_required(VERSION 3.10.2)
project(SimpleMassDuel VERSION 0.1.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(BehaviorTreeV3)

find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND BOOST_LOCAL_LIB
    ${PROJECT_SOURCE_DIR}/lib/libboost/libboost_exception.a
    ${PROJECT_SOURCE_DIR}/lib/libboost/libboost_filesystem.a
    ${PROJECT_SOURCE_DIR}/lib/libboost/libboost_filesystem.so
    ${PROJECT_SOURCE_DIR}/lib/libboost/libboost_iostreams.a
    ${PROJECT_SOURCE_DIR}/lib/libboost/libboost_iostreams.so
    ${PROJECT_SOURCE_DIR}/lib/libboost/libboost_program_options.a
    ${PROJECT_SOURCE_DIR}/lib/libboost/libboost_program_options.so
    ${PROJECT_SOURCE_DIR}/lib/libboost/libboost_system.a
    ${PROJECT_SOURCE_DIR}/lib/libboost/libboost_system.so
#   /usr/lib/x86_64-linux-gnu/libboost_exception.a
#   /usr/lib/x86_64-linux-gnu/libboost_filesystem.a
#   /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
#   /usr/lib/x86_64-linux-gnu/libboost_iostreams.a
#   /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
#   /usr/lib/x86_64-linux-gnu/libboost_program_options.a
#   /usr/lib/x86_64-linux-gnu/libboost_program_options.so
#   /usr/lib/x86_64-linux-gnu/libboost_system.a
#   /usr/lib/x86_64-linux-gnu/libboost_system.so
)

list(APPEND MASSDUEL_NEED_SRC
    ${PROJECT_SOURCE_DIR}/src/mass_point.cpp
    ${PROJECT_SOURCE_DIR}/src/mass_bt_nodes.cpp
)

add_library(massduel_v1 STATIC ${MASSDUEL_NEED_SRC})
target_include_directories(massduel_v1 PUBLIC ${PROJECT_SOURCE_DIR}/include)

list(APPEND DEMO_NEED_LIB
    BT::behaviortree_cpp_v3
    massduel_v1
    ${Boost_LIBRARIES}
    ${BOOST_LOCAL_LIB}
    ${CMAKE_THREAD_LIBS_INIT}
    ${CMAKE_DL_LIBS}
)

list(APPEND DRAW_NEED_LIB
    BT::behaviortree_cpp_v3
    massduel_v1
    ${Boost_LIBRARIES}
    ${BOOST_LOCAL_LIB}
    ${CMAKE_THREAD_LIBS_INIT}
    ${CMAKE_DL_LIBS}
)

add_executable(demo ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_compile_definitions(demo PUBLIC)
target_link_libraries(demo PUBLIC ${DEMO_NEED_LIB})


set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g2 -ggdb -fstack-protector-all -march=native -fnon-call-exceptions -lboost_program_options")
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -fstack-protector-all -march=native -fnon-call-exceptions")
set(CMAKE_CXX_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")