#include "gantry_gui/widgets/config_widget.hpp"

namespace gantry_gui {

ConfigWidget::ConfigWidget(rclcpp::Node::SharedPtr _node,
                           const std::vector<Axis> &axes, QWidget *parent)
    : QGroupBox(parent), node_(_node), axes_(axes) {
  setTitle("Config");
  QGridLayout *layout = new QGridLayout(this);

  QWidget *label;
  int col = 1;

  label = new QLabel("Homing", this);
  layout->addWidget(label, 0, col++, Qt::AlignCenter);

  label = new QLabel("Max Speed", this);
  layout->addWidget(label, 0, col++, Qt::AlignCenter);

  label = new QLabel("Max Accel", this);
  layout->addWidget(label, 0, col++, Qt::AlignCenter);

  int rows = static_cast<int>(axes_.size());
  for (int i = 0; i < rows; ++i) {
    col = 0;
    int row = i + 1;
    const Axis axis = axes_[i];
    QLabel *axis_label = new QLabel(axis_name(axis), this);
    axis_label->setStyleSheet("font-weight: bold;");
    layout->addWidget(axis_label, row, col++);

    HomingFrame *homing_widget = new HomingFrame(node_, this);
    homing_widgets_[axis] = homing_widget;
    homing_widget->InitGoHomeClient(go_home_service_name(axis));
    homing_widget->InitSetHomeClient(set_home_service_name(axis));
    // connect(homing_widget, &HomingFrame::GoHomeClicked, this,
    //         [=]() { emit GoHomeClicked(axis); });
    // connect(homing_widget, &HomingFrame::SetHomeClicked, this,
    //       [=](double value) { emit SetHomeClicked(axis, value); });
    layout->addWidget(homing_widget, row, col++);

    DoubleSpinboxFrame *max_speed = new DoubleSpinboxFrame(node_, this);
    max_speed_widgets_[axis] = max_speed;
    max_speed->InitLoadClient(get_max_speed_service_name(axis));
    max_speed->InitSaveClient(set_max_speed_service_name(axis));
    layout->addWidget(max_speed, row, col++);

    DoubleSpinboxFrame *max_accel = new DoubleSpinboxFrame(node_, this);
    max_accel_widgets_[axis] = max_accel;
    max_accel->InitLoadClient(get_max_accel_service_name(axis));
    max_accel->InitSaveClient(set_max_accel_service_name(axis));
    layout->addWidget(max_accel, row, col++);

    layout->setSizeConstraint(QLayout::SetFixedSize);

    connect(this, &ConfigWidget::LoadAllRequested, this,
            [max_speed, max_accel]() {
              max_speed->CallLoadService();
              max_accel->CallLoadService();
            });
  }
}

}  // namespace gantry_gui
