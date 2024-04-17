#pragma once
#include <QMainWindow>
#include <QWidget>
#include <QtUiTools>
#include <gantry_msgs/msg/motor_position.hpp>
#include <gantry_msgs/msg/motor_status.hpp>
#include <gantry_msgs/msg/motor_velocity.hpp>
#include <gantry_msgs/srv/set_home_position.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gantry_gui/common.hpp"
#include "gantry_gui/manual_control_node.hpp"
#include "gantry_gui/widgets/config_widget.hpp"
#include "gantry_gui/widgets/homing_frame.hpp"
#include "gantry_gui/widgets/position_widget.hpp"

namespace Ui {
class ManualControlWidget;
}

namespace gantry_gui {
class ManualControlGUI : public QWidget, public rclcpp::Node {
  Q_OBJECT

 public:
  explicit ManualControlGUI(QWidget *parent = nullptr,
                            QApplication *app = nullptr);
  void Init();
  void MoveDistance(Axis, double distance);
  void MoveVelocity(Axis, double velocity);
  ~ManualControlGUI();
 signals:
  void PositionChanged(double value, Axis);
  void VelocityChanged(double value, Axis);
  void ErrorRaised(QString title, QString text);
 public slots:
  void OnControlPadPressed(Axis, Direction);
  void OnControlPadReleased(Axis, Direction);

 protected:
  void closeEvent(QCloseEvent *event) override;

 private:
  struct Vector3d {
    double x;
    double y;
    double z;
  };
  void InitSubscriptions();
  void InitPublishers();

  void StopVelocityTimers();
  Mode mode_{Mode::kUnset};
  double mode_value_{0.0};
  std::mutex mutex_;
  Vector3d velocity_target;

  PositionWidget *position_widget_ = nullptr;
  HomingFrame *homing_widget_ = nullptr;
  ConfigWidget *config_widget_ = nullptr;
  QApplication *app_ = nullptr;

  std::map<Axis, rclcpp::TimerBase::SharedPtr> velocity_timers_;

  std::map<Axis,
           rclcpp::Subscription<gantry_msgs::msg::MotorPosition>::SharedPtr>
      position_subs_;
  std::map<Axis,
           rclcpp::Subscription<gantry_msgs::msg::MotorVelocity>::SharedPtr>
      velocity_subs_;
  std::map<Axis, rclcpp::Subscription<gantry_msgs::msg::MotorStatus>::SharedPtr>
      status_subs_;
  std::map<Axis, rclcpp::Publisher<gantry_msgs::msg::MotorPosition>::SharedPtr>
      relative_position_pubs_;
  std::map<Axis, rclcpp::Publisher<gantry_msgs::msg::MotorVelocity>::SharedPtr>
      velocity_pubs_;
};
}  // namespace gantry_gui
