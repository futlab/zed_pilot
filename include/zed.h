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
    sl::Camera camera;
    Zed();
    sl::ERROR_CODE inline open() { camera.open(parameters); }
    static sl::DeviceProperties zedFromSN(unsigned int serialNumber);
};

#endif // ZED_H
