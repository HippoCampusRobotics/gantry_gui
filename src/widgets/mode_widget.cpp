#include "gantry_gui/widgets/mode_widget.hpp"

namespace gantry_gui {
ModeWidget::ModeWidget(QWidget *parent) : QGroupBox(parent) {
  setTitle("Mode");
  QGridLayout *layout = new QGridLayout(this);

  QButtonGroup *button_group = new QButtonGroup(this);

  distance_button_ = new QRadioButton("Distance", this);
  button_group->addButton(distance_button_);
  connect(distance_button_, &QRadioButton::toggled, this, [this](bool checked) {
    if (checked) {
      emit ModeChanged(Mode::kDistance, distance_spinbox_->value());
    }
    distance_spinbox_->setEnabled(checked);
  });
  layout->addWidget(distance_button_, 0, 0);

  velocity_button_ = new QRadioButton("Velocity", this);
  connect(velocity_button_, &QRadioButton::toggled, this, [this](bool checked) {
    if (checked) {
      emit ModeChanged(Mode::kVelocity, velocity_spinbox_->value());
    }
    velocity_spinbox_->setEnabled(checked);
  });
  button_group->addButton(velocity_button_);
  layout->addWidget(velocity_button_, 1, 0);

  QHBoxLayout *hbox = new QHBoxLayout();
  distance_spinbox_ = new QDoubleSpinBox(this);
  distance_spinbox_->setDecimals(3);
  distance_spinbox_->setSingleStep(0.001);
  distance_spinbox_->setValue(0.1);
  distance_spinbox_->setEnabled(false);
  connect(distance_spinbox_, qOverload<double>(&QDoubleSpinBox::valueChanged),
          this,
          [this](double value) { emit ValueChanged(Mode::kDistance, value); });
  hbox->addWidget(distance_spinbox_);

  distance_unit_label = new QLabel("[m]", this);
  hbox->addWidget(distance_unit_label);

  layout->addLayout(hbox, 0, 1);

  hbox = new QHBoxLayout();
  velocity_spinbox_ = new QDoubleSpinBox(this);
  velocity_spinbox_->setDecimals(3);
  velocity_spinbox_->setSingleStep(0.001);
  velocity_spinbox_->setValue(0.1);
  velocity_spinbox_->setEnabled(false);
  hbox->addWidget(velocity_spinbox_);

  velocity_unit_label = new QLabel("[m/s]", this);
  hbox->addWidget(velocity_unit_label);

  layout->addLayout(hbox, 1, 1);

  connect(this, &ModeWidget::ModeChanged, this, [](Mode mode, double value) {
    RCLCPP_INFO(get_logger(), "Mode changed: %s = %lf", mode_name(mode), value);
  });
}
}  // namespace gantry_gui
