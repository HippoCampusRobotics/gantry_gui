#pragma once

#include <QtWidgets>

#include "gantry_gui/common.hpp"
#include "gantry_gui/widgets/label_unit_widget.hpp"

namespace gantry_gui {
class PositionWidget : public QGroupBox {
  Q_OBJECT
 public:
  PositionWidget(const std::vector<Axis> &axes, QWidget *parent = nullptr);
 public slots:
  void SetPosition(double value, Axis);
  void SetVelocity(double value, Axis);

 private:
  std::vector<Axis> axes_;
  std::map<Axis, LabelUnitWidget *> position_widgets_;
  std::map<Axis, LabelUnitWidget *> velocity_widgets_;
};
}  // namespace gantry_gui
