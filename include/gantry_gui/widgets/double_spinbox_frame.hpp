#pragma once

#include <QtWidgets>
#include <gantry_msgs/srv/get_float_drive.hpp>
#include <gantry_msgs/srv/set_float_drive.hpp>

#include "gantry_gui/common.hpp"

namespace gantry_gui {

class DoubleSpinboxFrame : public QFrame {
  Q_OBJECT
 public:
  DoubleSpinboxFrame(rclcpp::Node::SharedPtr _node, QWidget *parent = nullptr);
  ~DoubleSpinboxFrame() {}
  void InitLoadClient(const std::string &name);
  void InitSaveClient(const std::string &name);
 public slots:
  double GetValue() const;
  void SetValue(double v, int decimals = 3);
  void CallLoadService();
  void CallSaveService(double value);
 signals:
  void LoadClicked();
  void LoadResponseReceived(bool success, double value);
  void LoadTimedOut();

  void SaveClicked(double value);
  void SaveResponseReceived(bool success);
  void SaveTimedOut();

 private:
  QLabel *label_ = nullptr;
  QDoubleSpinBox *spinbox_ = nullptr;
  QPushButton *load_button_ = nullptr;
  QPushButton *save_button_ = nullptr;
  QPushButton *reset_button_ = nullptr;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<gantry_msgs::srv::GetFloatDrive>::SharedPtr load_client_;
  rclcpp::Client<gantry_msgs::srv::SetFloatDrive>::SharedPtr save_client_;
  rclcpp::TimerBase::SharedPtr load_timer_;
  rclcpp::TimerBase::SharedPtr save_timer_;
};

}  // namespace gantry_gui
