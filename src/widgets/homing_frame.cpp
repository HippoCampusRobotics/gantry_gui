#include "gantry_gui/widgets/homing_frame.hpp"

namespace gantry_gui {

HomingFrame::HomingFrame(rclcpp::Node::SharedPtr _node, QWidget *parent)
    : QFrame(parent), node_(_node) {
  setFrameStyle(QFrame::Panel);
  setFrameShadow(QFrame::Sunken);

  QVBoxLayout *layout = new QVBoxLayout(this);

  go_home_button_ =
      new QPushButton(QIcon::fromTheme("go-home"), "Go Home", this);
  connect(go_home_button_, &QPushButton::clicked, this,
          [=]() { emit GoHomeClicked(); });
  connect(this, &HomingFrame::GoHomeClicked, this,
          &HomingFrame::CallGoHomeService);
  connect(this, &HomingFrame::GoHomeTimedOut, this, [this]() {
    ShowError("Service timed out", TimeoutText(go_home_client_));
    EnableGoHome(true);
  });
  connect(this, &HomingFrame::GoHomeResponseReceived, this,
          [this](bool success) {
            if (!success) {
              ShowError("Service failed", "Service call failed.");
            }
            EnableGoHome(true);
          });
  layout->addWidget(go_home_button_);

  spinbox_ = new QDoubleSpinBox(this);
  spinbox_->setDecimals(3);
  spinbox_->setSingleStep(0.001);
  layout->addWidget(spinbox_);

  set_home_button_ = new QPushButton("Set Position", this);
  connect(set_home_button_, &QPushButton::clicked, this,
          [=]() { emit SetHomeClicked(GetValue()); });
  connect(this, &HomingFrame::SetHomeClicked, this,
          &HomingFrame::CallSetHomeService);
  connect(this, &HomingFrame::SetHomeTimedOut, this, [this]() {
    ShowError("Service timed out", TimeoutText(set_home_client_));
    EnableSetHome(true);
  });
  connect(this, &HomingFrame::SetHomeResponseReceived, this,
          [this](bool success) {
            if (!success) {
              ShowError("Service failed.", "Service call failed.");
            }
            EnableSetHome(true);
          });
  layout->addWidget(set_home_button_);
}

void HomingFrame::InitGoHomeClient(const std::string &name) {
  if (!node_) {
    return;
  }
  if (go_home_client_) {
    RCLCPP_WARN(
        get_logger(),
        "GoHome client seems to exist already. But a new one is created.");
  }
  go_home_client_ = node_->create_client<std_srvs::srv::Trigger>(name);
}

void HomingFrame::InitSetHomeClient(const std::string &name) {
  if (!node_) {
    return;
  }
  if (set_home_client_) {
    RCLCPP_WARN(
        get_logger(),
        "SetHome client already exists, but creation of new one is requested.");
  }
  set_home_client_ =
      node_->create_client<gantry_msgs::srv::SetHomePosition>(name);
}

void HomingFrame::CallGoHomeService() {
  if (!pointers_valid(node_, go_home_client_, go_home_button_)) {
    return;
  }
  go_home_client_->prune_pending_requests();
  cancel_timer(go_home_timer_);

  using ServiceT = std_srvs::srv::Trigger;
  using FutureT = rclcpp::Client<ServiceT>::SharedFuture;
  using RequestT = ServiceT::Request;
  EnableGoHome(false);
  auto req = std::make_shared<RequestT>();
  auto client_callback = [this](FutureT future) {
    emit GoHomeResponseReceived(future.get()->success);
    cancel_timer(go_home_timer_);
  };
  FutureT future =
      go_home_client_->async_send_request(req, client_callback).future;

  if (go_home_timer_) {
    go_home_timer_->cancel();
  }
  go_home_timer_ = node_->create_wall_timer(timeout_duration, [this, future]() {
    cancel_timer(go_home_timer_);
    if (future.wait_for(zero_duration) != std::future_status::ready) {
      emit GoHomeTimedOut();
    }
  });
}

void HomingFrame::CallSetHomeService(double _value) {
  if (!pointers_valid(node_, set_home_client_, set_home_button_)) {
    return;
  }
  set_home_client_->prune_pending_requests();
  cancel_timer(set_home_timer_);

  using ServiceT = gantry_msgs::srv::SetHomePosition;
  using FutureT = rclcpp::Client<ServiceT>::SharedFuture;
  using RequestT = ServiceT::Request;

  EnableSetHome(false);
  auto req = std::make_shared<RequestT>();
  req->position.position = _value;
  auto client_callback = [this](FutureT future) {
    emit SetHomeResponseReceived(future.get()->success);
    cancel_timer(set_home_timer_);
  };
  FutureT future =
      set_home_client_->async_send_request(req, client_callback).future;
  if (set_home_timer_) {
    set_home_timer_->cancel();
  }
  set_home_timer_ =
      node_->create_wall_timer(timeout_duration, [this, future]() {
        cancel_timer(set_home_timer_);
        if (future.wait_for(zero_duration) != std::future_status::ready) {
          emit SetHomeTimedOut();
        }
      });
}

double HomingFrame::GetValue() const { return spinbox_->value(); }

void HomingFrame::ShowError(QString title, QString text) {
  QMessageBox box;
  box.critical(nullptr, title, text);
}
}  // namespace gantry_gui
