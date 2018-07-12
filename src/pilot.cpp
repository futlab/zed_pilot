#include <opencv2/imgproc.hpp>

#include "../include/pilot.h"

void Pilot::setVelocitySP(const Twist &twist)
{
    if(attitudeMode) {
        float roll = twist.linear[0] * 0.3;
        float pitch = twist.linear[1] * 0.3;
        Quaternionf a =
                AngleAxisf(roll, Vector3f::UnitX()) *
                AngleAxisf(pitch, Vector3f::UnitY()) *
                yawAttitude;
        float thrust = -twist.linear[2];
        signalAttitudeSP(a, thrust);
    } else
        signalVelocitySP(twist);
}

Pilot::Pilot() :
    mode(PILOT_PASSIVE), lastArmed(false), attitudeMode(false),
    linearVelocityLimit(0.5), yawSpeedLimit(0.5),
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
    const auto &l = twist.linear;
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

void Pilot::onState(bool connected, bool armed, bool guided, const std::string &pcMode)
{
    Twist t;
    if (!lastArmed && armed)
        resetTracking();

    if (pcMode != lastPCMode) {
        if (pcMode == "OFFBOARD" || pcMode == "GUIDED") {
            targetPose = lastPose;
            mode = PILOT_STABILIZED;
        } else if (pcMode == "GUIDED_NOGPS") {
            attitudeMode = true;
            targetPose = lastPose;
            mode = PILOT_STABILIZED;
        } else {
            mode = PILOT_PASSIVE;
            twist = Twist();
            attitudeMode = false;
        }
    }

    lastPCMode = pcMode;
    lastArmed = armed;
    lastConnected = lastConnected;
}

void Pilot::onCameraPose(const Pose &pose, unsigned int confidence)
{
    lastPose = pose;
    poseConfidence = confidence;
    switch (mode) {
    case PILOT_PASSIVE:
        setVelocitySP(twist);
        break;
    case PILOT_STABILIZED:
        twist.linear = (targetPose.position - pose.position) * 0.4;
        auto norm = twist.linear.norm();
        if (norm > linearVelocityLimit)
            twist.linear /= (norm / linearVelocityLimit);
        setVelocitySP(twist);
        break;
    }
}

void Pilot::onAttitude(const Quaternionf &attitude)
{
    auto euler = attitude.toRotationMatrix().eulerAngles(0, 1, 2);
    yawAttitude = AngleAxisf(euler[2], Vector3f::UnitZ());
}
