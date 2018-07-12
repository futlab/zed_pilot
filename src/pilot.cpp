#include <opencv2/imgproc.hpp>

#include "../include/pilot.h"

Pilot::Pilot() :
    mode(PILOT_PASSIVE), lastArmed(false),
    linearVelocityLimit(0.5), yawSpeedLimit(0.5)
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
        if (pcMode == "OFFBOARD") {
            targetPose = lastPose;
            mode = PILOT_STABILIZED;
        } else {
            mode = PILOT_PASSIVE;
            twist = Twist();
        }
    }

    lastPCMode = pcMode;
    lastArmed = armed;
    lastConnected = lastConnected;
}

void Pilot::onPose(const Pose &pose, unsigned int confidence)
{
    lastPose = pose;
    switch (mode) {
    case PILOT_PASSIVE:
        updateVelocitySP(twist);
        break;
    case PILOT_STABILIZED:
        twist.linear = (targetPose.position - pose.position) * 0.2;
        auto norm = twist.linear.norm();
        if (norm > linearVelocityLimit)
            twist.linear /= (norm / linearVelocityLimit);
        updateVelocitySP(twist);
        break;
    }
}
