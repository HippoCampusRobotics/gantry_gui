#pragma once
#include <QtWidgets>
#include <gantry_gui/widgets/buttons/get_float_button.hpp>
#include <gantry_gui/widgets/buttons/set_float_button.hpp>
#include <gantry_gui/widgets/buttons/trigger_button.hpp>

#include "gantry_gui/common.hpp"

namespace gantry_gui {

class GetSetWidget : public QFrame {
  Q_OBJECT
 public:
  GetSetWidget(rclcpp::Node::SharedPtr _node, QWidget *parent = nullptr)
      : QFrame(parent), node_(_node) {
    setFrameStyle(QFrame::Panel);
    setFrameShadow(QFrame::Sunken);
    QVBoxLayout *layout = new QVBoxLayout(this);

    label_ = new QLabel("-", this);
    layout->addWidget(label_, 0, Qt::AlignHCenter);

    spinbox_ = new QDoubleSpinBox(this);
    spinbox_->setDecimals(3);
    spinbox_->setSingleStep(0.001);
    layout->addWidget(spinbox_);

    QHBoxLayout *button_layout = new QHBoxLayout();

    get_button_ = new GetFloatButton(_node, this);
    connect(get_button_, &GetFloatButton::clicked, this, [this]() {
      auto req = std::make_shared<gantry_msgs::srv::GetFloatDrive::Request>();
      get_button_->CallService(req);
    });
    connect(
        get_button_->signal_,
        qOverload<gantry_msgs::srv::GetFloatDrive::Response::SharedPtr>(
            &ServiceButtonSignal::ResponseReceived),
        this,
        [this](gantry_msgs::srv::GetFloatDrive::Response::SharedPtr response) {
          if (response->success) {
            spinbox_->setValue(response->driveside_value);
          } else {
            auto client = get_button_->Client();
            show_error("Service failed", failed_text(client.lock()));
          }
        });
    button_layout->addWidget(get_button_);

    set_button_ = new SetFloatButton(_node, this);
    button_layout->addWidget(set_button_);
    connect(set_button_, &SetFloatButton::clicked, this, [this]() {
      auto req = std::make_shared<gantry_msgs::srv::SetFloatDrive::Request>();
      req->driveside_value = spinbox_->value();
      set_button_->CallService(req);
    });
    connect(
        set_button_->signal_,
        qOverload<gantry_msgs::srv::SetFloatDrive::Response::SharedPtr>(
            &ServiceButtonSignal::ResponseReceived),
        this,
        [this](gantry_msgs::srv::SetFloatDrive::Response::SharedPtr response) {
          if (response->success) {
            auto req =
                std::make_shared<gantry_msgs::srv::GetFloatDrive::Request>();
            get_button_->CallService(req);
          } else {
            auto client = set_button_->Client();
            show_error("Service failed", failed_text(client.lock()));
          }
        });

    reset_button_ = new TriggerButton(_node, this);
    // reset_button_->setIcon(QIcon::fromTheme("edit-undo"));
    reset_button_->setText("Reset");
    button_layout->addWidget(reset_button_);
    connect(reset_button_, &TriggerButton::clicked, this, [this]() {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      reset_button_->CallService(req);
    });
    connect(
        reset_button_->signal_,
        qOverload<std_srvs::srv::Trigger::Response::SharedPtr>(
            &ServiceButtonSignal::ResponseReceived),
        this, [this](std_srvs::srv::Trigger::Response::SharedPtr response) {
          if (response->success) {
            RCLCPP_INFO(node_->get_logger(),
                        "Called Reset. Getting new value.");
            auto req =
                std::make_shared<gantry_msgs::srv::GetFloatDrive::Request>();
            get_button_->CallService(req);
          } else {
            auto client = reset_button_->Client();
            show_error("Service failed", failed_text(client.lock()));
          }
        });
    layout->addLayout(button_layout);
  }

  void SetSetButtonServiceName(const std::string &s) {
    set_button_->CreateClient(s);
  }
  void SetGetButtonServiceName(const std::string &s) {
    get_button_->CreateClient(s);
  }
  void SetResetButtonServiceName(const std::string &s) {
    reset_button_->CreateClient(s);
  }
 public slots:
  // double GetValue() const;
  // void SetValue(double v, int decimals = 3);
  void CallGet() {
    auto req = std::make_shared<gantry_msgs::srv::GetFloatDrive::Request>();
    get_button_->CallService(req);
  }

 private:
  QLabel *label_ = nullptr;
  QDoubleSpinBox *spinbox_ = nullptr;
  GetFloatButton *get_button_ = nullptr;
  SetFloatButton *set_button_ = nullptr;
  TriggerButton *reset_button_ = nullptr;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace gantry_gui
