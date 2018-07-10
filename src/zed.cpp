#include "zed.h"

Zed::Zed()
{
    parameters.sdk_verbose = true;
    parameters.coordinate_units = sl::UNIT_METER;
    parameters.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;

    trackingParameters.enable_spatial_memory = true;
}

sl::DeviceProperties Zed::zedFromSN(unsigned int serialNumber) {
    sl::DeviceProperties prop;
    auto f = sl::Camera::getDeviceList();
    for (auto &it : f) {
        if (it.serial_number == serialNumber && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
            prop = it;
    }
    return prop;
}
