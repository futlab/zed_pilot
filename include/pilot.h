#ifndef PILOT_H
#define PILOT_H
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>

using namespace Eigen;
using namespace std;

enum PilotMode
{
    PILOT_PASSIVE,
    PILOT_STABILIZED
};

struct Pose
{
    Quaternionf orientation;
    Vector3f position;
};

struct Twist
{
    Vector3f linear, angular;
    Twist() : linear(Vector3f::Zero()), angular(Vector3f::Zero()) {}
};

class Pilot
{
private:
    PilotMode mode;
    Pose lastPose, targetPose;
    Twist twist;
    bool lastArmed, lastConnected;
    string lastPCMode;
    float linearVelocityLimit, yawSpeedLimit;
    bool attitudeMode;
    int poseConfidence;
    void setVelocitySP(const Twist &twist);
    Quaternionf yawAttitude;
public:
    Pilot();
    void drawState(cv::Mat &stateImage);

    // Slots
    void onState(bool connected, bool armed, bool guided, const std::string &pcMode);
    void onCameraPose(const Pose &pose, unsigned int confidence);
    void onAttitude(const Quaternionf &attitude);

    // Signals
    function<void()> resetTracking;
    function<void(const Twist &)> signalVelocitySP;
    function<void(const Quaternionf &, float)> signalAttitudeSP;
};

#endif // PILOT_H
