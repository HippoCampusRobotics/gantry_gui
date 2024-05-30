
#include <QApplication>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

#include "gantry_gui/manual_control_gui.hpp"

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  qRegisterMetaType<gantry_gui::Axis>("gantry_gui::Axis");
  qRegisterMetaType<gantry_gui::Axis>("Axis");
  qRegisterMetaType<gantry_gui::Direction>("gantry_gui::Direction");
  qRegisterMetaType<gantry_gui::Direction>("Direction");

  rclcpp::init(argc, argv);
  rclcpp::experimental::executors::EventsExecutor exec;
  auto gui_node = std::make_shared<gantry_gui::ManualControlGUI>(nullptr, &app);
  exec.add_node(gui_node);
  gui_node->Init();
  std::thread ros_thread = std::thread([&exec]() { exec.spin(); });
  gui_node->show();
  app.exec();
  RCLCPP_INFO(gantry_gui::get_logger(), "app.exec() finished.");
  rclcpp::shutdown();
  RCLCPP_INFO(gantry_gui::get_logger(), "exec.cancel() finished.");
  ros_thread.join();
  return 0;
}
