message(STATUS "src This is BINARY dir" ${PROJECT_BINARY_DIR})
message(STATUS "src This is SOURCE dir" ${PROJECT_SOURCE_DIR})

# List demo linked libraries
list(APPEND ALL_NEED_LIB
	BT::behaviortree_cpp_v3
    ${Boost_LIBRARIES}
	cppzmq
    ${CMAKE_THREAD_LIBS_INIT}
    ${CMAKE_DL_LIBS}
)

list(APPEND DEMO2D_NEED_LIB
    mass_point_v2
	mass_duel_2d_v2
	mass_bt_nodes_v2
	${ALL_NEED_LIB}
)

list(APPEND DEMO3D_NEED_LIB
    mass_point_v2
	mass_bt_nodes_v2
	xtensor
    ${ALL_NEED_LIB}
)

# Add Executable Demo2D & Demo3D
add_executable(demo2d ${SRC}/main_2d.cpp)
target_compile_definitions(demo2d PUBLIC _USE_MATH_DEFINES)
target_link_libraries(demo2d PUBLIC ${DEMO2D_NEED_LIB})

add_executable(demo3d ${SRC}/main_3d.cpp)
target_compile_definitions(demo3d PUBLIC _USE_MATH_DEFINES)
target_link_libraries(demo3d PUBLIC ${DEMO3D_NEED_LIB})