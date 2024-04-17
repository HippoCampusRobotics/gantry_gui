#include "gantry_gui/widgets/position_widget.hpp"

namespace gantry_gui {

PositionWidget::PositionWidget(const std::vector<Axis> &axes, QWidget *parent)
    : QGroupBox(parent), axes_(axes) {
  setTitle("Position/Velocity");
  QGridLayout *layout = new QGridLayout(this);
  QWidget *label;

  label = new QLabel("Position", this);
  layout->addWidget(label, 0, 1);

  label = new QLabel("Velocity", this);
  layout->addWidget(label, 0, 2);

  int rows = static_cast<int>(axes_.size());

  for (int i = 0; i < rows; ++i) {
    const Axis axis = axes_.at(i);
    QLabel *axis_label = new QLabel(axis_name(axis), this);
    axis_label->setStyleSheet("font-weight: bold;");
    layout->addWidget(axis_label, i + 1, 0);

    LabelUnitWidget *p_widget = new LabelUnitWidget(this);
    p_widget->SetUnit("[m]");
    p_widget->SetLabel("-");
    position_widgets_[axis] = p_widget;
    layout->addWidget(p_widget, i + 1, 1);

    LabelUnitWidget *v_widget = new LabelUnitWidget(this);
    v_widget->SetUnit("[m/s]");
    v_widget->SetLabel("-");
    velocity_widgets_[axis] = v_widget;
    layout->addWidget(v_widget, i + 1, 2);
  }
}

void PositionWidget::SetPosition(double value, Axis axis) {
  QString text = QString::number(value, 'f', 3).rightJustified(7);
  position_widgets_.at(axis)->SetLabel(text);
}
void PositionWidget::SetVelocity(double value, Axis axis) {
  QString text = QString::number(value, 'f', 3).rightJustified(7);
  velocity_widgets_.at(axis)->SetLabel(text);
}

}  // namespace gantry_gui
