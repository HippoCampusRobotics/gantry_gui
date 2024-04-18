#pragma once

#include <qobjectdefs.h>

#include <gantry_gui/metatype_registration.hpp>
#include <gantry_gui/widgets/buttons/service_button.hpp>
#include <gantry_msgs/srv/get_float_drive.hpp>

namespace gantry_gui {
class GetFloatButton : public ServiceButton<gantry_msgs::srv::GetFloatDrive> {
 public:
  GetFloatButton(rclcpp::Node::SharedPtr _node, QWidget *parent = nullptr)
      : ServiceButton(_node, parent) {
    // setIcon(QIcon::fromTheme("view-refresh"));
    setText("Get");
  }
};
}  // namespace gantry_gui
   //
Q_DECLARE_METATYPE(gantry_msgs::srv::GetFloatDrive::Request::SharedPtr);
Q_DECLARE_METATYPE(gantry_msgs::srv::GetFloatDrive::Response::SharedPtr);

extern template int
qMetaTypeId<gantry_msgs::srv::GetFloatDrive::Request::SharedPtr>();
extern template int
qMetaTypeId<gantry_msgs::srv::GetFloatDrive::Response::SharedPtr>();
