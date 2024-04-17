#pragma once
#include <QMetaType>

template <typename T>
class MetaTypeRegistration {
 public:
  MetaTypeRegistration() {
    qRegisterMetaType<T>();
    qRegisterMetaTypeStreamOperators<T>();
  }
};
