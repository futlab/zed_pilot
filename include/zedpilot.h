#ifndef ZED_H
#define ZED_H
#include <string>
#include <memory>
#include <sl/Camera.hpp>
#include <opencv2/core.hpp>
#ifdef USE_GST
#include <pipeline.h>
#endif

#include "pilot.h"

class ZedPilot
{
private:
    std::chrono::steady_clock::time_point lastGrabTime, nextStateImageTime, svoRestartTime;
    std::unique_ptr<sl::Mat> slLeftImage, slRightImage;
    cv::Mat leftImage, rightImage;
    cv::Mat stateImage, stateImage4;
    cv::Size stateImageSize;
#ifdef USE_GST
    std::unique_ptr<Pipeline> transmitter;
#endif
    void processFrame();
    void processStateImage();
    void enableRecording();
    void processZedPose();
protected:
    sl::Pose pose;
    bool svoRecordingEnabled;
    size_t svoFramesRecorded, svoRecordNumber;
    Pilot pilot;
    std::chrono::steady_clock::duration stateImagePeriod, svoMaxDuration;
    std::string videoUdpTarget, svoOutputPrefix;
    virtual void publishPose(sl::Pose &) {}
    virtual void publishStateImage(const cv::Mat &stateImage);
    virtual void fatal(const std::string &message);
    virtual void warn(const std::string &message);
    virtual void info(const std::string &message);
    virtual void infoOnce(const std::string &message);
    virtual void debug(const std::string &message);
    virtual void publishVelositySP(const Twist &twist) {}
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
    void shutdown();
};

#endif // ZED_H
