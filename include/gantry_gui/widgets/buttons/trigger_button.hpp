#pragma once

#include <gantry_gui/metatype_registration.hpp>
#include <gantry_gui/widgets/buttons/service_button.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace gantry_gui {

class TriggerButton : public ServiceButton<std_srvs::srv::Trigger> {
 public:
  TriggerButton(rclcpp::Node::SharedPtr _node, QWidget *parent = nullptr)
      : ServiceButton(_node, parent) {}
};

}  // namespace gantry_gui
   //
Q_DECLARE_METATYPE(std_srvs::srv::Trigger::Response::SharedPtr);
Q_DECLARE_METATYPE(std_srvs::srv::Trigger::Request::SharedPtr);

extern template int qMetaTypeId<std_srvs::srv::Trigger::Request::SharedPtr>();
extern template int qMetaTypeId<std_srvs::srv::Trigger::Response::SharedPtr>();
