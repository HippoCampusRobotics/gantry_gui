#pragma once

#include <QtWidgets>

#include "gantry_gui/common.hpp"

namespace gantry_gui {

class ModeWidget : public QGroupBox {
  Q_OBJECT
 public:
  ModeWidget(QWidget *parent = nullptr);
 signals:
  void ModeChanged(Mode mode, double value);
  void ValueChanged(Mode mode, double value);

 private:
  std::map<int, Mode> button_id_map_{{0, Mode::kDistance},
                                     {1, Mode::kVelocity}};
  QRadioButton *distance_button_ = nullptr;
  QRadioButton *velocity_button_ = nullptr;
  QDoubleSpinBox *distance_spinbox_ = nullptr;
  QDoubleSpinBox *velocity_spinbox_ = nullptr;
  QLabel *distance_unit_label = nullptr;
  QLabel *velocity_unit_label = nullptr;
};
}  // namespace gantry_gui
