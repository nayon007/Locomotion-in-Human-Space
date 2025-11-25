cmake_minimum_required(VERSION 3.5)
project(go2_d1_wbc_eval)
find_package(ament_cmake REQUIRED)
install(PROGRAMS
  go2_d1_wbc_eval/wbc_router.py
  go2_d1_wbc_eval/wbc_logger.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
ament_package()
