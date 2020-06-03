cmake_minimum_required(VERSION 2.8.3)
project(usb_relay_wsn)

find_package(catkin REQUIRED COMPONENTS roslint message_generation std_msgs)

add_message_files(FILES
  RelayState.msg
  RelayStates.msg
)

add_service_files(FILES
  SetRelay.srv
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

catkin_install_python(PROGRAMS
  src/ros_relay_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Check package for pep8 style, add a test to fail on violations.
roslint_python()
roslint_add_test()
