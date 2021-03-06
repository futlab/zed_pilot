#include <opencv2/imgproc.hpp>

#include "../include/pilot.h"


Pilot::Pilot() :
    mode(PILOT_PASSIVE), targetPose{ Quaternionf::Identity(), Vector3f::Zero() },
    currentLinearCmd(Vector3f::Zero()), currentAngularCmd(Vector3f::Zero()),
    lastArmed(false),
    linearVelocityLimit(0.5), yawSpeedLimit(0.5), attitudeMode(false),
    poseConfidence(0)
{

}

void Pilot::drawState(cv::Mat &stateImage)
{
    const cv::Size size = stateImage.size();
    cv::Point modePoint(size.width / 2 - 20, size.height / 2 + 30);
    switch (mode) {
    case PILOT_PASSIVE:
        cv::putText(stateImage, "P", modePoint, cv::FONT_HERSHEY_PLAIN, 7, cv::Scalar(0, 0, 0), 3);
        break;
    case PILOT_STABILIZED:
        cv::putText(stateImage, "S", modePoint, cv::FONT_HERSHEY_PLAIN, 7, cv::Scalar(0, 150, 0), 3);
        break;
    }
    //cv::putText(stateImage, to_string(poseConfidence), cv::Point(modePoint.x + 40, modePoint.y), cv::FONT_HERSHEY_PLAIN, 5, cv::Scalar(0, poseConfidence * 2, 0));
    const auto &l = currentLinearCmd;
    if (l.any()) {
        Vector2f l2, l2p;
        l2 << l[0], l[1];
        l2 *= size.height / 2 * linearVelocityLimit;
        l2p << l2[1], -l2[0];

        Vector2f center(size.width / 2, size.height / 2);
        Vector2f pf1 = center + l2, pf2 = center + l2 * 0.5 + l2p * 0.5, pf3 = center + l2 * 0.5 - l2p * 0.5;
        cv::Point p1((int)pf1[0], (int)pf1[1]), p2((int)pf2[0], (int)pf2[1]), p3((int)pf3[0], (int)pf3[1]);

        cv::line(stateImage, p1, p2, cv::Scalar(0, 0, 255), 2);
        cv::line(stateImage, p1, p3, cv::Scalar(0, 0, 255), 2);
        cv::line(stateImage, p3, p2, cv::Scalar(0, 0, 255), 2);
    }
}

void Pilot::reset()
{
    resetTracking();
    targetPose.position = Vector3f::Zero();
    targetPose.orientation = Quaternionf::Identity();
    currentLinearCmd = Vector3f::Zero();
    currentAngularCmd = Vector3f::Zero();
}

void Pilot::onState(bool connected, bool armed, bool guided, const std::string &pcMode)
{
    if (!lastArmed && armed)
        reset();

    /*if (pcMode != lastPCMode) {
        if (pcMode == "ALT_HOLD") {
            reset();
            mode = PILOT_STABILIZED;
        } else if (pcMode == "OFFBOARD" || pcMode == "GUIDED") {
            reset();
            mode = PILOT_STABILIZED;
        } else if (pcMode == "GUIDED_NOGPS") {
            reset();
            attitudeMode = true;
            mode = PILOT_STABILIZED;
        } else {
            mode = PILOT_PASSIVE;
            currentLinearCmd = Vector3f::Zero();
            currentAngularCmd = Vector3f::Zero();
            attitudeMode = false;
        }
    }*/

    lastPCMode = pcMode;
    lastArmed = armed;
    lastConnected = connected;
}

void Pilot::onControl(int value)
{
    if (value != lastValue) {
        switch (value) {
        case 1:
        case 2:
            reset();
            mode = PILOT_STABILIZED;
            break;
        case 0:
            mode = PILOT_PASSIVE;
            //currentLinearCmd = Vector3f::Zero();
            //currentAngularCmd = Vector3f::Zero();
            attitudeMode = false;
            break;
        }
    }
    lastValue = value;
}

void Pilot::onCameraPose(const Pose &pose, int confidence)
{
    lastPose = pose;
    poseConfidence = confidence;
    switch (mode) {
    case PILOT_PASSIVE:
        /*currentLinearCmd *= 0.9f;
        currentAngularCmd *= 0.9f;
        setVelocitySP(currentLinearCmd, currentAngularCmd);*/
        break;
    case PILOT_STABILIZED: // pose and targetPose in camera frame
        currentLinearCmd = (targetPose.position - pose.position) * 0.4f;
        auto norm = currentLinearCmd.norm();
        if (norm > linearVelocityLimit)
            currentLinearCmd *= linearVelocityLimit / norm;
        setVelocitySP(controlMatrix * currentLinearCmd, currentAngularCmd);
        break;
    }
}

void Pilot::setShift(float x, float y, float z)
{
    targetPose = lastPose;
    Vector3f shift; shift << x, y, z;
    targetPose.position += shift;
}

void Pilot::addShift(float x, float y, float z)
{
    Vector3f shift; shift << x, y, z;
    targetPose.position += shift;
}


void Pilot::setVelocitySP(const Vector3f &linear, const Vector3f &angular) // twist in FC frame
{
    if(attitudeMode) {
        float roll = linear[0] * 0.3f;
        float pitch = linear[1] * 0.3f;
        Quaternionf a =
                AngleAxisf(roll, Vector3f::UnitX()) *
                AngleAxisf(pitch, Vector3f::UnitY()) *
                yawAttitude;
        float thrust = -linear[2];
        signalAttitudeSP(a, thrust);
    } else
        signalVelocitySP(linear, angular);
}

void Pilot::onAttitude(const Quaternionf &attitude)
{
    auto euler = attitude.toRotationMatrix().eulerAngles(0, 1, 2);
    yawAttitude = AngleAxisf(euler[2], Vector3f::UnitZ());
}

