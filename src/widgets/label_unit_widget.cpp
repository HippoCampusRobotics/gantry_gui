#include "gantry_gui/widgets/label_unit_widget.hpp"

namespace gantry_gui {

LabelUnitWidget::LabelUnitWidget(QWidget *parent) : QFrame(parent) {
  QHBoxLayout *layout = new QHBoxLayout(this);

  label_ = new QLabel("label", this);
  label_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  layout->addWidget(label_);

  unit_ = new QLabel("unit", this);
  layout->addWidget(unit_);
}

void LabelUnitWidget::SetLabel(QString s) {
  if (!label_) {
    return;
  }
  label_->setText(s);
}

void LabelUnitWidget::SetUnit(QString s) {
  if (!unit_) {
    return;
  }
  unit_->setText(s);
}
}  // namespace gantry_gui
