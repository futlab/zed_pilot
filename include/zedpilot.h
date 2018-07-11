#ifndef ZED_H
#define ZED_H
#include <string>
#include <sl/Camera.hpp>

#include "pilot.h"

class ZedPilot
{
private:
    sl::Pose pose;
    std::chrono::steady_clock::time_point lastGrabTime;
protected:
    Pilot pilot;
    virtual void warn(const std::string &message);
    virtual void info(const std::string &message);
    virtual void infoOnce(const std::string &message);
    virtual void debug(const std::string &message);
    virtual void publishPose(sl::Pose &) {}
    void infoOnce(sl::ERROR_CODE code);
    void warn(sl::ERROR_CODE code);
    unsigned int serialNumber;
    sl::InitParameters parameters;
    sl::TrackingParameters trackingParameters;
    sl::RuntimeParameters runtimeParameters;
    sl::Camera camera;
    void open();
    void open(const std::string &svoFileName);
    void open(sl::RESOLUTION resolution, int zedId);
    void reopen();
    sl::DeviceProperties zedFromSN();
    void grab();
    inline sl::ERROR_CODE enableTracking() { return camera.enableTracking(trackingParameters); }
    int checkCameraReady();
public:
    ZedPilot();
};

#endif // ZED_H
