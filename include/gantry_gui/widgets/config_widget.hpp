#pragma once

#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>

#include "gantry_gui/common.hpp"
#include "gantry_gui/widgets/get_set_widget.hpp"
#include "gantry_gui/widgets/homing_frame.hpp"

namespace gantry_gui {

class ConfigWidget : public QGroupBox {
  Q_OBJECT
 public:
  ConfigWidget(rclcpp::Node::SharedPtr, const std::vector<Axis> &axes,
               QWidget *parent);
 signals:
  void LoadAllRequested();

 private:
  GetSetWidget *CreateMaxSpeedWidget(Axis);
  GetSetWidget *CreateMaxAccelWidget(Axis);
  rclcpp::Node::SharedPtr node_;
  std::vector<Axis> axes_;
  std::map<Axis, HomingFrame *> homing_widgets_;
  std::map<Axis, GetSetWidget *> max_speed_widgets_;
  std::map<Axis, GetSetWidget *> max_accel_widgets_;
};

}  // namespace gantry_gui
