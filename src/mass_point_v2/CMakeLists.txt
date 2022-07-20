set(MASSPOINT_NEED_SRC
	${SRC}/mass_point_v2/math_2d.cpp
	${SRC}/mass_point_v2/point_2d.cpp
	${SRC}/mass_point_v2/line_2d.cpp
	${SRC}/mass_point_v2/circle_2d.cpp
	${SRC}/mass_point_v2/polygon_2d.cpp
)

set(MASSPOINT_NEED_HDR
	${INC}/mass_point_v2/math_2d.h
	${INC}/mass_point_v2/point_2d.h
	${INC}/mass_point_v2/line_2d.h
	${INC}/mass_point_v2/circle_2d.h
	${INC}/mass_point_v2/polygon_2d.h
	${INC}/mass_point_v2/mass_point_2d.hpp
)

source_group("Header Files" FILES ${MASSPOINT_NEED_HDR})

add_library(mass_point_v2 STATIC ${MASSPOINT_NEED_SRC} ${MASSPOINT_NEED_HDR})
target_compile_definitions(mass_point_v2 PUBLIC _USE_MATH_DEFINES)
target_include_directories(mass_point_v2 PUBLIC ${INC})