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
      : QFrame(parent) {
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
    button_layout->addWidget(get_button_);

    set_button_ = new SetFloatButton(_node, this);
    set_button_->CreateClient("test");
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
            auto client = reset_button_->Client();
            show_error("Service ", failed_text(client.lock()));
          }
        });

    reset_button_ = new TriggerButton(_node, this);
    reset_button_->setIcon(QIcon::fromTheme("edit-undo"));
    button_layout->addWidget(reset_button_);
    connect(
        reset_button_->signal_,
        qOverload<std_srvs::srv::Trigger::Response::SharedPtr>(
            &ServiceButtonSignal::ResponseReceived),
        this, [this](std_srvs::srv::Trigger::Response::SharedPtr response) {
          if (response->success) {
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

 private:
  QLabel *label_ = nullptr;
  QDoubleSpinBox *spinbox_ = nullptr;
  GetFloatButton *get_button_ = nullptr;
  SetFloatButton *set_button_ = nullptr;
  TriggerButton *reset_button_ = nullptr;
};

}  // namespace gantry_gui
