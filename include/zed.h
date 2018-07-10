#ifndef ZED_H
#define ZED_H
#include <sl/Camera.hpp>

class Zed
{
protected:
    void warn();
public:
    unsigned int serialNumber;
    sl::InitParameters parameters;
    sl::TrackingParameters trackingParameters;
    sl::RuntimeParameters runtimeParameters;
    sl::Camera camera;
    Zed();
    sl::ERROR_CODE inline open() { return camera.open(parameters); }
    static sl::DeviceProperties zedFromSN(unsigned int serialNumber);
    inline sl::ERROR_CODE grab() { return camera.grab(runtimeParameters); }
    inline sl::ERROR_CODE enableTracking() { return camera.enableTracking(trackingParameters); }
    int checkCameraReady();
};

#endif // ZED_H
