#include <gantry_gui/widgets/buttons/get_float_button.hpp>

static const int kTriggerResponseMetaTypeId =
    qRegisterMetaType<gantry_msgs::srv::GetFloatDrive::Request::SharedPtr>();
static const int kTriggerRequestMetaTypeId =
    qRegisterMetaType<gantry_msgs::srv::GetFloatDrive::Response::SharedPtr>();

template int qMetaTypeId<gantry_msgs::srv::GetFloatDrive::Request::SharedPtr>();
template int
qMetaTypeId<gantry_msgs::srv::GetFloatDrive::Response::SharedPtr>();
