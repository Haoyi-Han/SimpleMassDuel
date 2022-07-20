cmake_minimum_required(VERSION 3.10.2)
project(SimpleMassDuel VERSION 0.1.9)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_DEBUG_MODE "Build a Debug version" ON)
option(BUILD_UNIT_TESTS "Build Unit Tests" ON)

set(INC ${PROJECT_SOURCE_DIR}/include)
set(SRC ${PROJECT_SOURCE_DIR}/src)
set(3RD ${PROJECT_SOURCE_DIR}/3rdparty)
set(TEST ${PROJECT_SOURCE_DIR}/tests)

#---- Find Packages ----
set(Boost_USE_STATIC_LIBS ON)
#set(Boost_USE_STATIC_RUNTIME ON)
find_package(Boost COMPONENTS program_options exception context REQUIRED QUIET)
#if(Boost_FOUND)
#    include_directories(${Boost_INCLUDE_DIRS})
#    link_directories(${Boost_LIBRARY_DIRS})
#else()
#    message(WARNING "boost not found.")
#endif()
find_package(cppzmq)
find_package(xtl)
find_package(xtensor)
find_package(BehaviorTreeV3)

#---- Add Library MassPointV2 & MassDuel2DV2 & MassBtNodesV2 ----
add_subdirectory(${SRC}/mass_point_v2)
add_subdirectory(${SRC}/mass_duel_2d_v2)
add_subdirectory(${SRC}/mass_bt_nodes_v2)

#---- Add Executable Demo2D & Demo3D ----
add_subdirectory(${SRC})

#---- Build Tests ----
if(BUILD_UNIT_TESTS)
	find_package(GTest)
	add_subdirectory(${TEST})
endif()

#---- Build Configuration ----
if(BUILD_DEBUG_MODE)
	set(CMAKE_BUILD_TYPE "Debug")
	set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g2 -ggdb -fstack-protector-all -march=native -fnon-call-exceptions -lboost_program_options")
	# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -fstack-protector-all -march=native -fnon-call-exceptions")
	set(CMAKE_CXX_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
else()
	set(CMAKE_BUILD_TYPE "Release")
endif()