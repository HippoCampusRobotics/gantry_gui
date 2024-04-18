#include "gantry_gui/manual_control_gui.hpp"

#include "gantry_gui/widgets/config_widget.hpp"
#include "gantry_gui/widgets/control_pad.hpp"
#include "gantry_gui/widgets/get_set_widget.hpp"
#include "gantry_gui/widgets/homing_frame.hpp"
#include "gantry_gui/widgets/mode_widget.hpp"
#include "gantry_gui/widgets/position_widget.hpp"

namespace gantry_gui {
ManualControlGUI::ManualControlGUI(QWidget *parent, QApplication *app)
    : QWidget(parent), Node("manual_control_gui"), app_(app) {}
void ManualControlGUI::Init() {
  QVBoxLayout *vlayout = new QVBoxLayout();

  ModeWidget *mode_widget = new ModeWidget(this);
  vlayout->addWidget(mode_widget);
  connect(mode_widget, &ModeWidget::ModeChanged, this,
          [this](Mode mode, double value) {
            mode_ = mode;
            mode_value_ = value;
            if (mode_ != Mode::kVelocity) {
              StopVelocityTimers();
            }
          });

  ControlPad *control_pad = new ControlPad(this);
  connect(control_pad, &ControlPad::ButtonReleased, this,
          &ManualControlGUI::OnControlPadReleased);
  connect(control_pad, &ControlPad::ButtonPressed, this,
          &ManualControlGUI::OnControlPadPressed);
  vlayout->addWidget(control_pad);

  position_widget_ = new PositionWidget({Axis::kX, Axis::kY, Axis::kZ}, this);
  position_widget_->SetPosition(100.1234, Axis::kX);
  vlayout->addWidget(position_widget_);

  vlayout->addStretch();

  QHBoxLayout *hlayout = new QHBoxLayout();
  hlayout->addLayout(vlayout);

  vlayout = new QVBoxLayout();
  config_widget_ = new ConfigWidget(shared_from_this(),
                                    {Axis::kX, Axis::kY, Axis::kZ}, this);
  vlayout->addWidget(config_widget_);
  vlayout->addStretch();
  emit config_widget_->LoadAllRequested();

  hlayout->addLayout(vlayout);
  hlayout->addStretch();

  setLayout(hlayout);

  connect(this, &ManualControlGUI::PositionChanged, this,
          [=](double value, Axis axis) {
            position_widget_->SetPosition(value, axis);
          });
  connect(this, &ManualControlGUI::VelocityChanged, this,
          [=](double value, Axis axis) {
            position_widget_->SetVelocity(value, axis);
          });
  InitPublishers();
  InitSubscriptions();
}

void ManualControlGUI::InitSubscriptions() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(1);

  std::vector<Axis> axes = {Axis::kX, Axis::kY, Axis::kZ};

  for (const auto axis : axes) {
    topic = position_topic_name(axis);
    position_subs_[axis] = create_subscription<gantry_msgs::msg::MotorPosition>(
        topic, qos, [=](const gantry_msgs::msg::MotorPosition::SharedPtr msg) {
          emit PositionChanged(msg->position, axis);
        });
    topic = velocity_topic_name(axis);
    velocity_subs_[axis] = create_subscription<gantry_msgs::msg::MotorVelocity>(
        topic, qos, [=](const gantry_msgs::msg::MotorVelocity::SharedPtr msg) {
          emit VelocityChanged(msg->velocity, axis);
        });
  }
}

void ManualControlGUI::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(1);
  std::vector<Axis> axes = {Axis::kX, Axis::kY, Axis::kZ};
  for (const auto axis : axes) {
    topic = setpoint_topic_name(axis, Mode::kVelocity);
    velocity_pubs_[axis] =
        create_publisher<gantry_msgs::msg::MotorVelocity>(topic, qos);

    topic = setpoint_topic_name(axis, Mode::kDistance);
    relative_position_pubs_[axis] =
        create_publisher<gantry_msgs::msg::MotorPosition>(topic, qos);
  }
}

void ManualControlGUI::OnControlPadPressed(Axis _axis, Direction _direction) {
  double sign = (_direction == Direction::kForward) ? 1.0 : -1.0;
  switch (mode_) {
    case Mode::kDistance:
      MoveDistance(_axis, sign * mode_value_);
      break;
    case Mode::kVelocity:
      MoveVelocity(_axis, sign * mode_value_);
      break;
    default:
      break;
  }
}

void ManualControlGUI::StopVelocityTimers() {
  std::lock_guard<decltype(mutex_)> guard{mutex_};
  for (auto &[axis, timer] : velocity_timers_) {
    if (timer) {
      timer->cancel();
    }
    RCLCPP_INFO(get_logger(), "Killing velocity timer: %s", axis_name(axis));
    velocity_timers_[axis] = nullptr;
  }
}

void ManualControlGUI::OnControlPadReleased(Axis _axis, Direction) {
  if (mode_ == Mode::kVelocity) {
    MoveVelocity(_axis, 0.0);
  }
}

void ManualControlGUI::MoveDistance(Axis _axis, double _distance) {
  gantry_msgs::msg::MotorPosition msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.position = _distance;
  relative_position_pubs_.at(_axis)->publish(msg);
}

void ManualControlGUI::MoveVelocity(Axis _axis, double _velocity) {
  if (velocity_timers_[_axis]) {
    velocity_timers_[_axis]->cancel();
  }
  assert(
      (velocity_timers_[_axis].use_count() <= 1) &&
      "There should not be more than one reference. Indicates a memory leak.");
  velocity_timers_[_axis] =
      create_wall_timer(std::chrono::milliseconds(20),
                        [this, axis = _axis, velocity = _velocity]() {
                          std::lock_guard<decltype(mutex_)> guard{mutex_};
                          gantry_msgs::msg::MotorVelocity msg;
                          msg.header.stamp = now();
                          msg.header.frame_id = "map";
                          msg.velocity = velocity;
                          velocity_pubs_.at(axis)->publish(msg);
                        });
}

ManualControlGUI::~ManualControlGUI() {
  RCLCPP_INFO(get_logger(), "Hello from destructor");
}

void ManualControlGUI::closeEvent(QCloseEvent *event) {
  event->accept();
  RCLCPP_INFO(get_logger(), "Close event called!");
  // config_widget_->deleteLater();
  // position_widget_->deleteLater();
  auto children = findChildren<QObject *>();
  foreach (auto child, children) {
    child->deleteLater();
  }
}
}  // namespace gantry_gui
