# Copyright (C) 2023 Thies Lennart Alff
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
# USA

cmake_minimum_required(VERSION 3.5)
project(gantry_gui)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2 -fPIC)
add_compile_options(${CXX_FLAGS})
#set(CMAKE_INCLUDE_CURRENT_DIR ON)


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gantry_msgs REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5UiTools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include_directories(
  include
)

set(exec_name manual_control)
add_executable(${exec_name}
  include/gantry_gui/manual_control_gui.hpp
  src/manual_control_gui.cpp
  src/manual_control_node.cpp
  src/main.cpp
  include/gantry_gui/common.hpp
  include/gantry_gui/widgets/homing_frame.hpp
  include/gantry_gui/widgets/control_pad.hpp
  include/gantry_gui/widgets/mode_widget.hpp
  include/gantry_gui/widgets/label_unit_widget.hpp
  include/gantry_gui/widgets/position_widget.hpp
  include/gantry_gui/widgets/config_widget.hpp
  include/gantry_gui/widgets/buttons/get_float_button.hpp
  include/gantry_gui/widgets/buttons/service_button.hpp
  include/gantry_gui/widgets/buttons/set_float_button.hpp
  include/gantry_gui/widgets/buttons/trigger_button.hpp
  include/gantry_gui/widgets/get_set_widget.hpp
  src/widgets/homing_frame.cpp
  src/widgets/control_pad.cpp
  src/widgets/mode_widget.cpp
  src/widgets/label_unit_widget.cpp
  src/widgets/position_widget.cpp
  src/widgets/config_widget.cpp
  src/widgets/buttons/get_float_button.cpp
  src/widgets/buttons/set_float_button.cpp
  src/widgets/buttons/trigger_button.cpp
  resources/manual_control.qrc
)
target_link_libraries(${exec_name}
  Qt5::Core
  Qt5::Widgets
  Qt5::UiTools
)
target_include_directories(${exec_name} PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/${exec_name}_autogen/include
)
ament_target_dependencies(${exec_name}
  rclcpp
  gantry_msgs
  std_srvs
)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# install shared resources
################################################################################
install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
