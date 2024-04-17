#include <gantry_gui/widgets/buttons/trigger_button.hpp>

static const int kTriggerResponseMetaTypeId =
    qRegisterMetaType<std_srvs::srv::Trigger::Request::SharedPtr>();
static const int kTriggerRequestMetaTypeId =
    qRegisterMetaType<std_srvs::srv::Trigger::Response::SharedPtr>();

template int qMetaTypeId<std_srvs::srv::Trigger::Request::SharedPtr>();
template int qMetaTypeId<std_srvs::srv::Trigger::Response::SharedPtr>();
