
#pragma once

#include <qobjectdefs.h>

#include <gantry_gui/metatype_registration.hpp>
#include <gantry_gui/widgets/buttons/service_button.hpp>
#include <gantry_msgs/srv/set_float_drive.hpp>

namespace gantry_gui {
class SetFloatButton : public ServiceButton<gantry_msgs::srv::SetFloatDrive> {
 public:
  SetFloatButton(rclcpp::Node::SharedPtr _node, QWidget *parent = nullptr)
      : ServiceButton(_node, parent) {
    // setIcon(QIcon::fromTheme("document-save"));
    setText("Set");
  }
};
}  // namespace gantry_gui
   //
Q_DECLARE_METATYPE(gantry_msgs::srv::SetFloatDrive::Request::SharedPtr);
Q_DECLARE_METATYPE(gantry_msgs::srv::SetFloatDrive::Response::SharedPtr);

extern template int
qMetaTypeId<gantry_msgs::srv::SetFloatDrive::Request::SharedPtr>();
extern template int
qMetaTypeId<gantry_msgs::srv::SetFloatDrive::Response::SharedPtr>();
