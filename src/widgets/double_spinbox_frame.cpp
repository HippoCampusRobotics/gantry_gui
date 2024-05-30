#include "gantry_gui/widgets/double_spinbox_frame.hpp"

#include <QLayout>
#include <QtWidgets>

namespace gantry_gui {
DoubleSpinboxFrame::DoubleSpinboxFrame(rclcpp::Node::SharedPtr _node,
                                       QWidget *parent)
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

  load_button_ = new QPushButton(this);
  load_button_->setIcon(QIcon::fromTheme("view-refresh"));
  connect(load_button_, &QPushButton::clicked, this,
          &DoubleSpinboxFrame::CallLoadService);
  connect(this, &DoubleSpinboxFrame::LoadTimedOut, this, [this]() {
    show_error("Service timed out", timeout_text(load_client_));
    enable_widget(load_button_, true);
    label_->setText("n/a");
  });
  connect(this, &DoubleSpinboxFrame::LoadResponseReceived, this,
          [this](bool success, double value) {
            enable_widget(load_button_, true);
            if (!success) {
              show_error("Service failed", failed_text(load_client_));
              label_->setText("n/a");
            } else {
              SetValue(value);
            }
          });

  QHBoxLayout *nested_layout = new QHBoxLayout();

  nested_layout->addWidget(load_button_);

  save_button_ = new QPushButton(this);
  save_button_->setIcon(QIcon::fromTheme("document-save"));
  connect(save_button_, &QPushButton::clicked, this,
          [this]() { CallSaveService(GetValue()); });
  connect(this, &DoubleSpinboxFrame::SaveTimedOut, this, [this]() {
    show_error("Service timed out", timeout_text(save_client_));
    enable_widget(save_button_, true);
  });
  connect(this, &DoubleSpinboxFrame::SaveResponseReceived, this,
          [this](bool success) {
            enable_widget(save_button_, true);
            if (!success) {
              show_error("Service failed", failed_text(save_client_));
            } else {
              emit load_button_->clicked();
            }
          });
  nested_layout->addWidget(save_button_);

  layout->addLayout(nested_layout, 1);
}

void DoubleSpinboxFrame::InitLoadClient(const std::string &name) {
  if (!node_) {
    return;
  }
  if (load_client_) {
    RCLCPP_WARN(
        get_logger(),
        "Load client already exsists, but creation of new one is requested.");
  }
  load_client_ = node_->create_client<gantry_msgs::srv::GetFloatDrive>(name);
}

void DoubleSpinboxFrame::InitSaveClient(const std::string &name) {
  if (!node_) {
    return;
  }
  if (save_client_) {
    RCLCPP_WARN(
        get_logger(),
        "Save client already exists, but creation of new one is requested.");
  }
  save_client_ = node_->create_client<gantry_msgs::srv::SetFloatDrive>(name);
}

void DoubleSpinboxFrame::CallLoadService() {
  if (!pointers_valid(node_, load_client_, load_button_)) {
    return;
  }
  load_client_->prune_pending_requests();
  cancel_timer(load_timer_);

  using ServiceT = gantry_msgs::srv::GetFloatDrive;
  using FutureT = rclcpp::Client<ServiceT>::SharedFuture;
  using RequestT = ServiceT::Request;

  enable_widget(load_button_, false);
  auto req = std::make_shared<RequestT>();
  auto client_callback = [this](FutureT future) {
    emit LoadResponseReceived(future.get()->success,
                              future.get()->driveside_value);
    cancel_timer(load_timer_);
  };
  FutureT future =
      load_client_->async_send_request(req, client_callback).future;

  if (load_timer_) {
    load_timer_->cancel();
  }
  load_timer_ = node_->create_wall_timer(timeout_duration, [this, future]() {
    cancel_timer(load_timer_);
    if (future.wait_for(zero_duration) != std::future_status::ready) {
      emit LoadTimedOut();
    }
  });
}

void DoubleSpinboxFrame::CallSaveService(double _value) {
  if (!pointers_valid(node_, save_client_, save_button_)) {
    return;
  }
  save_client_->prune_pending_requests();
  cancel_timer(save_timer_);

  using ServiceT = gantry_msgs::srv::SetFloatDrive;
  using FutureT = rclcpp::Client<ServiceT>::SharedFuture;
  using RequestT = ServiceT::Request;

  enable_widget(save_button_, false);
  auto req = std::make_shared<RequestT>();
  req->driveside_value = _value;
  auto client_callback = [this](FutureT future) {
    emit SaveResponseReceived(future.get()->success);
    cancel_timer(save_timer_);
  };
  FutureT future =
      save_client_->async_send_request(req, client_callback).future;
  if (save_timer_) {
    save_timer_->cancel();
  }
  save_timer_ = node_->create_wall_timer(timeout_duration, [this, future]() {
    cancel_timer(save_timer_);
    if (future.wait_for(zero_duration) != std::future_status::ready) {
      emit SaveTimedOut();
    }
  });
}

double DoubleSpinboxFrame::GetValue() const {
  if (!spinbox_) {
    QMessageBox::critical(nullptr, "Missing widget",
                          "No valid reference to SpinBox.");
    return std::numeric_limits<double>::quiet_NaN();
  }
  return spinbox_->value();
}

void DoubleSpinboxFrame::SetValue(double v, int decimals) {
  if (!label_) {
    QMessageBox::critical(nullptr, "Missing widget",
                          "No valid reference to label.");
    return;
  }
  QString text = QString::number(v, 'f', decimals);
  label_->setText(text);
}

}  // namespace gantry_gui
