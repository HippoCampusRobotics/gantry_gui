#include "gantry_gui/manual_control_node.hpp"

namespace gantry_gui {
ManualControlNode::ManualControlNode() : Node("manual_control") {
  RCLCPP_INFO(get_logger(), "Initialized");
}
}  // namespace gantry_gui
