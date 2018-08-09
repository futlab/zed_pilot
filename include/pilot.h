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

class Pilot
{
private:
    PilotMode mode;
    int lastValue = 0;
    Pose lastPose, targetPose;
    Vector3f currentLinearCmd, currentAngularCmd; // In camera frame
    bool lastArmed, lastConnected;
    string lastPCMode;
    float linearVelocityLimit, yawSpeedLimit;
    bool attitudeMode;
    int poseConfidence;
    void setVelocitySP(const Vector3f &linear, const Vector3f &angular);
    Quaternionf yawAttitude;
    Matrix3f controlMatrix;
public:
    inline void setControlMatrix(const Matrix3f &cm) { controlMatrix = cm; }
    Pilot();
    void drawState(cv::Mat &stateImage);
    void reset();

    // Slots
    void onState(bool connected, bool armed, bool guided, const std::string &pcMode);
    void onControl(int value);
    void onCameraPose(const Pose &pose, int confidence);
    void onAttitude(const Quaternionf &attitude);
    void setShift(float x, float y, float z);
    void addShift(float x, float y, float z);

    // Signals
    function<void()> resetTracking;
    function<void(const Vector3f &, const Vector3f &)> signalVelocitySP;
    function<void(const Quaternionf &, float)> signalAttitudeSP;
};

#endif // PILOT_H
