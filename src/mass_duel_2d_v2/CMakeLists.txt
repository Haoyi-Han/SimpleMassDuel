set(MASSDUEL_2D_NEED_SRC
	${SRC}/mass_duel_2d_v2/hinder.cpp
	${SRC}/mass_duel_2d_v2/search.cpp
)

set(MASSDUEL_2D_NEED_HDR
	${INC}/mass_duel_2d_v2/hinder.h
	${INC}/mass_duel_2d_v2/search.h
)

source_group("Header Files" FILES ${MASSDUEL_2D_NEED_HDR})

add_library(mass_duel_2d_v2 STATIC ${MASSDUEL_2D_NEED_SRC} ${MASSDUEL_2D_NEED_HDR})
target_compile_definitions(mass_duel_2d_v2 PUBLIC _USE_MATH_DEFINES)
target_include_directories(mass_duel_2d_v2 PUBLIC ${INC})
target_link_libraries(mass_duel_2d_v2 PUBLIC mass_point_v2)