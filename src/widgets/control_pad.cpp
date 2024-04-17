#include "gantry_gui/widgets/control_pad.hpp"

namespace gantry_gui {
ControlPad::ControlPad(QWidget *parent) : QGroupBox(parent) {
  setTitle("Control");
  QGridLayout *layout = new QGridLayout(this);

  auto button_factory = [this](const QString icon_name) {
    QPushButton *button =
        new QPushButton(QIcon::fromTheme(icon_name), "", this);
    button->setIconSize(QSize(32, 32));
    return button;
  };

  x_forward_button_ = button_factory("go-next");
  layout->addWidget(x_forward_button_, 1, 2);
  connect(x_forward_button_, &QPushButton::pressed, this,
          [=]() { emit ButtonPressed(Axis::kX, Direction::kForward); });
  connect(x_forward_button_, &QPushButton::released, this,
          [=]() { emit ButtonReleased(Axis::kX, Direction::kBackward); });

  x_backward_button_ = button_factory("go-previous");
  layout->addWidget(x_backward_button_, 1, 0);
  connect(x_backward_button_, &QPushButton::pressed, this,
          [=]() { emit ButtonPressed(Axis::kX, Direction::kBackward); });
  connect(x_backward_button_, &QPushButton::released, this,
          [=]() { emit ButtonReleased(Axis::kX, Direction::kBackward); });

  y_forward_button_ = button_factory("go-up");
  layout->addWidget(y_forward_button_, 0, 1);
  connect(y_forward_button_, &QPushButton::pressed, this,
          [=]() { emit ButtonPressed(Axis::kY, Direction::kForward); });
  connect(y_forward_button_, &QPushButton::released, this,
          [=]() { emit ButtonReleased(Axis::kY, Direction::kForward); });

  y_backward_button_ = button_factory("go-down");
  layout->addWidget(y_backward_button_, 2, 1);
  connect(y_backward_button_, &QPushButton::pressed, this,
          [=]() { emit ButtonPressed(Axis::kY, Direction::kBackward); });
  connect(y_backward_button_, &QPushButton::released, this,
          [=]() { emit ButtonReleased(Axis::kY, Direction::kBackward); });

  z_forward_button_ = button_factory("go-up");
  layout->addWidget(z_forward_button_, 0, 3);
  connect(z_forward_button_, &QPushButton::pressed, this,
          [=]() { emit ButtonPressed(Axis::kZ, Direction::kForward); });
  connect(z_forward_button_, &QPushButton::released, this,
          [=]() { emit ButtonReleased(Axis::kZ, Direction::kForward); });

  z_backward_button_ = button_factory("go-down");
  layout->addWidget(z_backward_button_, 2, 3);
  connect(z_backward_button_, &QPushButton::pressed, this,
          [=]() { emit ButtonPressed(Axis::kZ, Direction::kBackward); });
  connect(z_backward_button_, &QPushButton::released, this,
          [=]() { emit ButtonReleased(Axis::kZ, Direction::kBackward); });
  layout->setSizeConstraint(QLayout::SetFixedSize);
}
}  // namespace gantry_gui
