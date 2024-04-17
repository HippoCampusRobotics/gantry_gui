#pragma once
#include <QtCore>
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>

namespace gantry_gui {

constexpr std::chrono::milliseconds timeout_duration =
    std::chrono::milliseconds(1000);
constexpr std::chrono::milliseconds zero_duration =
    std::chrono::milliseconds(0);

enum class Axis {
  kX,
  kY,
  kZ,
  kRoll,
  kPitch,
  kYaw,
};

enum class Mode {
  kDistance,
  kVelocity,
  kUnset,
};

inline const char *mode_name(Mode mode) {
  switch (mode) {
    case Mode::kDistance:
      return "Distance";
    case Mode::kVelocity:
      return "Velocity";
    case Mode::kUnset:
      return "Unset";
  }
  throw std::runtime_error("Mode not defined.");
}

inline const char *axis_name(Axis axis) {
  switch (axis) {
    case Axis::kX:
      return "x";
    case Axis::kY:
      return "y";
    case Axis::kZ:
      return "z";
    case Axis::kRoll:
      return "roll";
    case Axis::kPitch:
      return "pitch";
    case Axis::kYaw:
      return "yaw";
    default:
      return "Unknown axis";
  }
}

enum class Direction {
  kForward,
  kBackward,
};

inline rclcpp::Logger get_logger() { return rclcpp::get_logger("gantry_gui"); }

inline void cancel_timer(rclcpp::TimerBase::SharedPtr _timer) {
  if (_timer) {
    _timer->cancel();
  }
}

inline void enable_widget(QWidget *_widget, bool _enable) {
  if (_widget) {
    _widget->setEnabled(_enable);
  }
}

template <typename ClientT>
bool pointers_valid(rclcpp::Node::SharedPtr _node, ClientT _client,
                    QWidget *_widget) {
  return _node && _client && _widget;
}

inline void show_error(QString title, QString text) {
  QMessageBox mbox;
  mbox.critical(nullptr, title, text);
}

template <typename ClientT>
inline QString timeout_text(ClientT client) {
  return QString("Timed out: '%1'")
      .arg(client ? client->get_service_name()
                  : "Invalid reference to service.");
}

template <typename ClientT>
inline QString failed_text(ClientT client) {
  return QString("Service call failed: '%1'")
      .arg(client ? client->get_service_name()
                  : "Invalid reference to service.");
}

inline std::string name_prefix(Axis axis) {
  return "/gantry/motor_" + std::string(axis_name(axis));
}

inline std::string position_topic_name(Axis axis) {
  return name_prefix(axis) + "/position";
}
inline std::string velocity_topic_name(Axis axis) {
  return name_prefix(axis) + "/velocity";
}
inline std::string status_topic_name(Axis axis) {
  return name_prefix(axis) + "/status";
}
inline std::string velocity_setpoint_topic_name(Axis axis) {
  return name_prefix(axis) + "/setpoint/velocity";
}
inline std::string relative_position_setpoint_topic_name(Axis axis) {
  return name_prefix(axis) + "/setpoint/relative_position";
}
inline std::string setpoint_topic_name(Axis axis, Mode mode) {
  switch (mode) {
    case Mode::kVelocity:
      return velocity_setpoint_topic_name(axis);
    case Mode::kDistance:
      return relative_position_setpoint_topic_name(axis);
    default:
      throw std::runtime_error(
          "Could not determine setpoint topic name for unhandled mode: " +
          std::string(mode_name(mode)));
  }
}
inline std::string go_home_service_name(Axis axis) {
  return name_prefix(axis) + "/start_homing";
}
inline std::string set_home_service_name(Axis axis) {
  return name_prefix(axis) + "/set_home_position";
}
inline std::string get_max_speed_service_name(Axis axis) {
  return name_prefix(axis) + "/get_max_speed";
}
inline std::string set_max_speed_service_name(Axis axis) {
  return name_prefix(axis) + "/set_max_speed";
}
inline std::string get_max_accel_service_name(Axis axis) {
  return name_prefix(axis) + "/get_max_accel";
}
inline std::string set_max_accel_service_name(Axis axis) {
  return name_prefix(axis) + "/set_max_accel";
}
inline std::string get_max_decel_service_name(Axis axis) {
  return name_prefix(axis) + "/get_max_decel";
}
inline std::string set_max_decel_service_name(Axis axis) {
  return name_prefix(axis) + "/set_max_decel";
}

}  // namespace gantry_gui
Q_DECLARE_METATYPE(gantry_gui::Axis)
Q_DECLARE_METATYPE(gantry_gui::Direction)
