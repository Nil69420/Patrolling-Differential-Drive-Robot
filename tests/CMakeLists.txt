ament_add_gtest(bt_core_actions_test bt_core_actions_test.cpp)
ament_add_gtest(navigation_contract_test navigation_contract_test.cpp)
ament_add_gtest(tracking_lifecycle_test tracking_lifecycle_test.cpp)

set(bt_test_targets
  bt_core_actions_test
  navigation_contract_test
  tracking_lifecycle_test
)

foreach(bt_test_target ${bt_test_targets})
  if(TARGET ${bt_test_target})
    ament_target_dependencies(${bt_test_target} ${dependencies})
    target_link_libraries(${bt_test_target} ${BTCPP_LINK_LIBRARIES})
  endif()
endforeach()