#pragma once

#include <qobjectdefs.h>

#include <QtWidgets>
#include <gantry_msgs/srv/set_home_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gantry_gui/common.hpp"

namespace gantry_gui {

class HomingFrame : public QFrame {
  Q_OBJECT

 public:
  HomingFrame(rclcpp::Node::SharedPtr _node, QWidget *parent = nullptr);
  double GetValue() const;
  void InitGoHomeClient(const std::string &name);
  void InitSetHomeClient(const std::string &name);
 signals:
  void GoHomeClicked();
  void SetHomeClicked(double position);
  void GoHomeResponseReceived(bool success);
  void SetHomeResponseReceived(bool success);
  void GoHomeTimedOut();
  void SetHomeTimedOut();
 public slots:
  inline void EnableGoHome(bool enable) {
    enable_widget(go_home_button_, enable);
  }
  inline void EnableSetHome(bool enable) {
    enable_widget(set_home_button_, enable);
  }
  void CallGoHomeService();
  void CallSetHomeService(double value);

 private:
  void InitClients();
  void InitTimers();
  void ShowError(QString title, QString text);
  template <typename T>
  inline QString TimeoutText(T client) {
    return QString("Timed out: '%1'")
        .arg(client ? client->get_service_name()
                    : "Invalid reference to service.");
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr go_home_client_;
  rclcpp::TimerBase::SharedPtr go_home_timer_;
  rclcpp::Client<gantry_msgs::srv::SetHomePosition>::SharedPtr set_home_client_;
  rclcpp::TimerBase::SharedPtr set_home_timer_;
  QPushButton *go_home_button_ = nullptr;
  QDoubleSpinBox *spinbox_ = nullptr;
  QPushButton *set_home_button_ = nullptr;
};

}  // namespace gantry_gui
