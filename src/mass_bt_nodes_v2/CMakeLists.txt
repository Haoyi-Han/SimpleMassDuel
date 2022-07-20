set(MASSBTNODES_NEED_SRC
	${SRC}/mass_bt_nodes_v2/transfer_nodes_2d.cpp
	${SRC}/mass_bt_nodes_v2/move_nodes_2d.cpp
	${SRC}/mass_bt_nodes_v2/mass_bt_nodes_2d.cpp
)

set(MASSBTNODES_NEED_HDR
	${INC}/mass_bt_nodes_v2/node_var_func_2d.hpp
	${INC}/mass_bt_nodes_v2/node_reg_init_2d.hpp
	${INC}/mass_bt_nodes_v2/transfer_nodes_2d.h
	${INC}/mass_bt_nodes_v2/move_nodes_2d.h
	${INC}/mass_bt_nodes_v2/mass_bt_nodes_2d.hpp
)
source_group("Header Files" FILES ${MASSBTNODES_NEED_HDR})

add_library(mass_bt_nodes_v2 STATIC ${MASSBTNODES_NEED_SRC} ${MASSBTNODES_NEED_HDR})
target_compile_definitions(mass_bt_nodes_v2 PUBLIC _USE_MATH_DEFINES)
target_include_directories(mass_bt_nodes_v2 PUBLIC ${INC})
target_link_libraries(mass_bt_nodes_v2 PUBLIC mass_point_v2 mass_duel_2d_v2)