#include <iostream>
#include "zedpilot.h"

using namespace std;

ZedPilot::ZedPilot() : serialNumber(0)
{
    parameters.sdk_verbose = true;
    parameters.coordinate_units = sl::UNIT_METER;
    parameters.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;

    trackingParameters.enable_spatial_memory = true;

    runtimeParameters.enable_depth = true;
}

void ZedPilot::warn(const std::string &message) { cout << "WARNING: " << message << endl; }
void ZedPilot::info(const string &message)      { cout << "INFO: " << message << endl; }
void ZedPilot::infoOnce(const string &message)  { cout << "INFO: " << message << endl; }
void ZedPilot::debug(const string &message)     { cout << "DEBUG: " << message << endl; }
void ZedPilot::warn(sl::ERROR_CODE code)        { warn(sl::toString(code).c_str()); }
void ZedPilot::infoOnce(sl::ERROR_CODE code)    { infoOnce(sl::toString(code).c_str()); }

void ZedPilot::open()
{
    sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
    while (err != sl::SUCCESS) {
        err = camera.open(parameters);
        warn(err);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    enableTracking();
}

void ZedPilot::open(const string &svoFileName)
{
    parameters.svo_input_filename = svoFileName.c_str();
    open();
}

void ZedPilot::open(sl::RESOLUTION resolution, int zedId)
{
    parameters.camera_resolution = resolution;
    if (serialNumber == 0)
        parameters.camera_linux_id = zedId;
    else {
        bool waiting_for_camera = true;
        while (waiting_for_camera) {
            sl::DeviceProperties prop = zedFromSN();
            if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE) {
                warn("ZED SN" + to_string(serialNumber) + " not detected ! Please connect this ZED");
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            } else {
                waiting_for_camera = false;
                parameters.camera_linux_id = prop.id;
            }
        }
    }
    open();
}

void ZedPilot::reopen()
{
    camera.close();

    info("Re-opening the ZED");
    sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
    while (err != sl::SUCCESS) {
        int id = checkCameraReady();
        if (id > 0) {
            parameters.camera_linux_id = id;
            err = camera.open(); // Try to initialize the ZED
            warn(err);
        } else info("Waiting for the ZED to be re-connected");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    camera.enableTracking();
}

sl::DeviceProperties ZedPilot::zedFromSN()
{
    sl::DeviceProperties prop;
    auto f = sl::Camera::getDeviceList();
    for (auto &it : f) {
        if (it.serial_number == serialNumber && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
            prop = it;
    }
    return prop;
}

void ZedPilot::grab()
{
    auto grabStatus = camera.grab(runtimeParameters);

    if (grabStatus != sl::ERROR_CODE::SUCCESS) { // Detect if a error occurred (for example: the zed have been disconnected) and re-initialize the ZED

        if (grabStatus == sl::ERROR_CODE_NOT_A_NEW_FRAME) {
            debug("Wait for a new image to proceed");
        } else infoOnce(grabStatus);

        this_thread::sleep_for(std::chrono::milliseconds(2));

        std::chrono::duration<double> duration = chrono::steady_clock::now() - lastGrabTime;
        if (duration.count() > 5)
            reopen();
        return;
    } else
        lastGrabTime = chrono::steady_clock::now();

    camera.getPosition(pose);
    publishPose(pose);
}

int ZedPilot::checkCameraReady()
{
    int id = -1;
    auto f = sl::Camera::getDeviceList();
    for (auto &it : f)
        if (it.serial_number == serialNumber && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
            id = it.id;
    return id;
}
