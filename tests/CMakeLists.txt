enable_testing()

add_library(test_tools INTERFACE)
target_include_directories(test_tools INTERFACE ${TEST}/include)
target_link_libraries(test_tools INTERFACE mass_point_v2)

set(TEST_SRC 
	${TEST}/mass_point_2d_test.cpp
	${TEST}/mass_duel_2d_test.cpp
	${TEST}/mass_bt_nodes_2d_test.cpp
)
	
add_executable(unittest ${TEST}/main.cpp ${TEST_SRC})
target_link_libraries(unittest PRIVATE mass_point_v2 mass_duel_2d_v2 mass_bt_nodes_v2 test_tools GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)
add_test(unittest unittest)
