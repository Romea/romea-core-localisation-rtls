catkin_add_gtest(${PROJECT_NAME}_test_simple_trialteration test_simple_trilateration.cpp )
# target_link_libraries(${PROJECT_NAME}_test_robot_to_human ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_test_simple_trialteration ${catkin_LIBRARIES} ${PROJECT_NAME})

# catkin_add_gtest(${PROJECT_NAME}_test_robot_to_human test/test_robot_to_human.cpp )
# target_link_libraries(${PROJECT_NAME}_test_robot_to_human ${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME}_test_robot_to_human ${catkin_LIBRARIES} robot_to_human_rtls_localisation_plugin_nodelet)

# catkin_add_gtest(${PROJECT_NAME}_test_robot_to_robot test/test_robot_to_robot.cpp )
# target_link_libraries(${PROJECT_NAME}_test_robot_to_robot ${PROJECT_NAME} ${catkin_LIBRARIES})
