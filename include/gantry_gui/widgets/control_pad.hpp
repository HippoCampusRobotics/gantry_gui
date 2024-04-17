#pragma once

#include <QtWidgets>

#include "gantry_gui/common.hpp"

namespace gantry_gui {
class ControlPad : public QGroupBox {
  Q_OBJECT
 public:
  ControlPad(QWidget *parent = nullptr);
 signals:
  void ButtonPressed(Axis, Direction);
  void ButtonReleased(Axis, Direction);

 private:
  QPushButton *x_forward_button_ = nullptr;
  QPushButton *x_backward_button_ = nullptr;
  QPushButton *y_forward_button_ = nullptr;
  QPushButton *y_backward_button_ = nullptr;
  QPushButton *z_forward_button_ = nullptr;
  QPushButton *z_backward_button_ = nullptr;
};
}  // namespace gantry_gui
