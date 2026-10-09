cmake_minimum_required(VERSION 3.16)
project(pingpong_launcher_sim)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)
add_library(pingpong_ball_launcher SHARED src/ball_launcher.cpp)
target_link_libraries(pingpong_ball_launcher PRIVATE
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
  ignition-transport11::core)
target_compile_options(pingpong_ball_launcher PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS pingpong_ball_launcher LIBRARY DESTINATION lib)
add_library(pingpong_physical_ball_monitor SHARED src/physical_ball_monitor.cpp)
target_link_libraries(pingpong_physical_ball_monitor PRIVATE
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
  ignition-transport11::core)
target_compile_options(pingpong_physical_ball_monitor PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS pingpong_physical_ball_monitor LIBRARY DESTINATION lib)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/launcher_control DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config models worlds docs DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md LICENSE NOTICE DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_control_math test/test_control_math.py)
  ament_add_pytest_test(test_model test/test_model.py)
  ament_add_pytest_test(test_ball_inventory test/test_ball_inventory.py)
  ament_add_pytest_test(test_physical_model test/test_physical_model.py)
endif()
ament_package()
