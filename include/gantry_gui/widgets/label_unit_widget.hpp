#pragma once

#include <QtWidgets>

#include "gantry_gui/common.hpp"

namespace gantry_gui {
class LabelUnitWidget : public QFrame {
  Q_OBJECT
 public:
  LabelUnitWidget(QWidget *parent = nullptr);
 public slots:
  void SetUnit(QString s);
  void SetLabel(QString s);

 private:
  QLabel *label_ = nullptr;
  QLabel *unit_ = nullptr;
};
}  // namespace gantry_gui
