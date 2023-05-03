# catkin_add_gtest(${PROJECT_NAME}_test_simple_trialteration test_simple_trilateration.cpp )
# target_link_libraries(${PROJECT_NAME}_test_robot_to_human ${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME}_test_simple_trialteration ${PROJECT_NAME})

# catkin_add_gtest(${PROJECT_NAME}_test_robot_to_human test/test_robot_to_human.cpp )
# target_link_libraries(${PROJECT_NAME}_test_robot_to_human ${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME}_test_robot_to_human robot_to_human_rtls_localisation_plugin_nodelet)

# catkin_add_gtest(${PROJECT_NAME}_test_robot_to_robot test/test_robot_to_robot.cpp )
# target_link_libraries(${PROJECT_NAME}_test_robot_to_robot ${PROJECT_NAME} ${catkin_LIBRARIES})


find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_simple_trialteration test_simple_trilateration.cpp)
target_link_libraries(${PROJECT_NAME}_test_simple_trialteration ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_simple_trialteration PRIVATE -std=c++17)
add_test(test ${PROJECT_NAME}_test_simple_trialteration)

add_executable(${PROJECT_NAME}_test_position_estimator test_position_estimator.cpp)
target_link_libraries(${PROJECT_NAME}_test_position_estimator  ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_position_estimator PRIVATE -std=c++17)
add_test(test ${PROJECT_NAME}_test_position_estimator)

add_executable(${PROJECT_NAME}_test_pose_estimator test_pose_estimator.cpp)
target_link_libraries(${PROJECT_NAME}_test_pose_estimator  ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_pose_estimator PRIVATE -std=c++17)
add_test(test ${PROJECT_NAME}_test_pose_estimator)