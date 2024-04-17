
#include <gantry_gui/widgets/buttons/set_float_button.hpp>

static const int kTriggerResponseMetaTypeId =
    qRegisterMetaType<gantry_msgs::srv::SetFloatDrive::Request::SharedPtr>();
static const int kTriggerRequestMetaTypeId =
    qRegisterMetaType<gantry_msgs::srv::SetFloatDrive::Response::SharedPtr>();

template int qMetaTypeId<gantry_msgs::srv::SetFloatDrive::Request::SharedPtr>();
template int
qMetaTypeId<gantry_msgs::srv::SetFloatDrive::Response::SharedPtr>();
