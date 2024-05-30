#pragma once
#include <QtWidgets>
#include <gantry_gui/common.hpp>
#include <gantry_msgs/srv/get_float_drive.hpp>
#include <gantry_msgs/srv/set_float_drive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace gantry_gui {
class ServiceButtonSignal : public QObject {
  Q_OBJECT
 public:
  ServiceButtonSignal() : QObject() {}
 signals:
  void ResponseReceived(std_srvs::srv::Trigger::Response::SharedPtr);
  void ResponseReceived(gantry_msgs::srv::SetFloatDrive::Response::SharedPtr);
  void ResponseReceived(gantry_msgs::srv::GetFloatDrive::Response::SharedPtr);
  void TimedOut();
};

template <typename ServiceT>
class ServiceButton : public QPushButton {
 public:
  ServiceButton(rclcpp::Node::SharedPtr _node, QWidget *parent = nullptr)
      : QPushButton(parent), node_(_node) {
    signal_ = new ServiceButtonSignal();
    connect(signal_, &ServiceButtonSignal::TimedOut, this,
            &ServiceButton::OnTimeout);
    connect(signal_,
            qOverload<typename ServiceT::Response::SharedPtr>(
                &ServiceButtonSignal::ResponseReceived),
            this, [this](typename ServiceT::Response::SharedPtr) {
              setEnabled(true);
            });
  }
  ~ServiceButton() { delete signal_; }
  void CreateClient(const std::string &name) {
    assert(node_);
    client_ = node_->create_client<ServiceT>(name);
  }
  typename rclcpp::Client<ServiceT>::ConstWeakPtr Client() const {
    return client_;
  }

 public:
  void CallService(typename ServiceT::Request::SharedPtr request) {
    assert(node_);
    assert(client_);
    client_->prune_pending_requests();
    CancelTimer();
    setDisabled(true);
    auto client_callback =
        [this](typename rclcpp::Client<ServiceT>::SharedFuture future) {
          signal_->ResponseReceived(future.get());
          CancelTimer();
        };
    typename rclcpp::Client<ServiceT>::SharedFuture future =
        client_->async_send_request(request, client_callback).future;
    if (timer_) {
      timer_->cancel();
    }
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000), [this, future]() {
          CancelTimer();
          if (future.wait_for(std::chrono::milliseconds(0)) !=
              std::future_status::ready) {
            signal_->TimedOut();
          }
        });
  };
  void OnTimeout() {
    show_error("Service timed out", timeout_text(client_));
    setDisabled(false);
  }

  typename ServiceT::Request::SharedPtr CreateRequest() {
    return std::make_shared<typename ServiceT::Request>();
  }

  ServiceButtonSignal *signal_ = nullptr;

 protected:
  inline void CancelTimer() {
    if (timer_) {
      timer_->cancel();
    }
  }
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  rclcpp::Node::SharedPtr node_;

 private:
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace gantry_gui
