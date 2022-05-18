cmake_minimum_required(VERSION 3.10.2)
project(SimpleMassDuel VERSION 0.1.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(BehaviorTreeV3)

find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND BOOST_LOCAL_LIB
    ${PROJECT_SOURCE_DIR}/lib/libboost_program_options.a
    ${PROJECT_SOURCE_DIR}/lib/libboost_program_options.so
)

list(APPEND NEED_SRC
    ${PROJECT_SOURCE_DIR}/src/mass_point.cpp
    ${PROJECT_SOURCE_DIR}/src/mass_bt_nodes.cpp
)

add_library(massduel STATIC ${NEED_SRC})
target_include_directories(massduel PUBLIC ${PROJECT_SOURCE_DIR}/include)

list(APPEND NEED_LIB
    BT::behaviortree_cpp_v3
    massduel
#   ${Boost_LIBRARIES}
    ${BOOST_LOCAL_LIB}
#   /usr/lib/x86_64-linux-gnu/libboost_program_options.a /usr/lib/x86_64-linux-gnu/libboost_program_options.so
    ${CMAKE_THREAD_LIBS_INIT}
    ${CMAKE_DL_LIBS}
)

add_executable(demo ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_compile_definitions(demo PUBLIC)
target_link_libraries(demo PUBLIC ${NEED_LIB})


set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g2 -ggdb -fstack-protector-all -march=native -fnon-call-exceptions -lboost_program_options")
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -fstack-protector-all -march=native -fnon-call-exceptions")
set(CMAKE_CXX_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")