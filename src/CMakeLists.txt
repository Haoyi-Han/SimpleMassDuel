message(STATUS "src This is BINARY dir" ${PROJECT_BINARY_DIR})
message(STATUS "src This is SOURCE dir" ${PROJECT_SOURCE_DIR})

set(SRC_LIST
    main.cpp
    mass_point.cpp
    mass_bt_nodes.cpp
)

add_executable(src_exec ${SRC_LIST})