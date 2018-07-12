// STL includes
#include <string>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#ifdef MSG_GEN
#include <zed_pilot/ZedState.h>
#else
#include <std_msgs/String.h>
#endif
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

// Local includes
#include "zedpilot.h"

using namespace std;

class ZedPilotNode : public ZedPilot
{
private:
    ros::NodeHandle nh, nhp;
    ros::Publisher odometryPub, velocitySpPub, statePub;
    ros::Subscriber mavStateSub;
    string odometryFrameId, baseFrameId, cameraFrameId;
    unique_ptr<tf2_ros::Buffer> tfBuffer;
    bool publishTf;
    tf2::Transform baseToSensor, baseTransform;
    tf2_ros::TransformBroadcaster transformOdomBroadcaster;
    ros::Time grabTime;
    geometry_msgs::TwistStamped velocitySP;

    void publishOdom(tf2::Transform base_transform, string odomFrame, ros::Time t);
    void publishTrackedFrame(tf2::Transform base_transform, tf2_ros::TransformBroadcaster &trans_br, string odometryTransformFrameId, ros::Time t);
    void publishState();
    void advertise();
    void subscribe();
    void MavStateCallback(const mavros_msgs::State::ConstPtr &state);
protected:
    void fatal(const std::string &message)      { ROS_FATAL("%s", message.c_str()); }
    void warn(const std::string &message)       { ROS_WARN("%s", message.c_str()); }
    void info(const std::string &message)       { ROS_INFO("%s", message.c_str()); }
    void infoOnce(const std::string &message)   { ROS_INFO_ONCE("%s", message.c_str()); }
    void debug(const std::string &message)      { ROS_DEBUG("%s", message.c_str()); }

    void publishPose(sl::Pose &pose);
    void publishVelositySP(const Twist &twist);
public:
    ZedPilotNode() : nhp("~") {}
    void init();
    void spin();
};



void ZedPilotNode::advertise()
{
    string odometryTopic = "odom";
    nhp.getParam("odometry_topic", odometryTopic);
    odometryPub = nh.advertise<nav_msgs::Odometry>(odometryTopic, 1);
    ROS_INFO_STREAM("Advertised on topic " << odometryTopic);

    string twistTopic = "mavros/setpoint_velocity/cmd_vel";
    velocitySpPub = nh.advertise<geometry_msgs::TwistStamped>(twistTopic, 1);
    ROS_INFO_STREAM("Advertised on topic " << twistTopic);

    string stateTopic = "state";
#ifdef GEM_MSG
    statePub = nh.advertise<zed_pilot::ZedState>(stateTopic, 1);
#else
    statePub = nh.advertise<std_msgs::String>(stateTopic, 1);
#endif
    ROS_INFO_STREAM("Advertised on topic " << stateTopic);
}

void ZedPilotNode::subscribe()
{
    mavStateSub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &ZedPilotNode::MavStateCallback, this);
}

void ZedPilotNode::MavStateCallback(const mavros_msgs::State::ConstPtr &state)
{
    pilot.onState(state->connected, state->armed, state->guided, state->mode);
}

void ZedPilotNode::init()
{
    // Defaults
    int
            resolution = sl::RESOLUTION_HD720,
            quality = sl::DEPTH_MODE_PERFORMANCE,
            sensingMode = sl::SENSING_MODE_STANDARD,
            rate = 60,
            gpuId = -1,
            zedId = 0,
            svoMaxRecord;

    string
            svoFilepath = "";

    double outFps;

    // Get parameters

    // Set  default coordinate frames
    // If unknown left and right frames are set in the same camera coordinate frame
    nhp.param<string>("odometry_frame", odometryFrameId, "odometry_frame");
    nhp.param<string>("base_frame", baseFrameId, "base_frame");
    nhp.param<string>("camera_frame", cameraFrameId, "camera_frame");

    nhp.getParam("video_udp_target", videoUdpTarget);
    nhp.getParam("svo_output_prefix", svoOutputPrefix);
    if (nhp.getParam("svo_max_record", svoMaxRecord))
        svoMaxDuration = chrono::seconds{svoMaxRecord};
    nhp.getParam("resolution", resolution);
    nhp.getParam("quality", quality);
    parameters.depth_mode = static_cast<sl::DEPTH_MODE>(quality);
    nhp.getParam("sensing_mode", sensingMode);
    runtimeParameters.sensing_mode = static_cast<sl::SENSING_MODE>(sensingMode);
    nhp.getParam("frame_rate", rate);
    parameters.camera_fps = rate;
    if (nhp.getParam("out_fps", outFps))
        stateImagePeriod = chrono::milliseconds{int(1000 / outFps)};
    string odometryDB;
    nhp.getParam("odometry_DB", odometryDB);
    trackingParameters.area_file_path = odometryDB.c_str();
    nhp.param<bool>("publish_tf", publishTf, false);

    nhp.getParam("gpu_id", gpuId);
    parameters.sdk_gpu_id = gpuId;
    nhp.getParam("zed_id", zedId);
    int tmpSn = 0;
    nhp.getParam("serial_number", tmpSn);
    if (tmpSn > 0) serialNumber = tmpSn;

    if (serialNumber > 0)
        ROS_INFO_STREAM("SN : " << serialNumber);

    nhp.param<string>("svo_filepath", svoFilepath, string());

    if (!svoFilepath.empty())
        open(svoFilepath);
    else
        open((sl::RESOLUTION)resolution, zedId);

    tfBuffer = make_unique<tf2_ros::Buffer>();
    advertise();
    subscribe();
}

void ZedPilotNode::spin()
{
    ros::Time oldT;
    ros::Rate loopRate(parameters.camera_fps);
    while(ros::ok()) {
        grabTime = ros::Time::now(); // Get current time

        grab();
        publishState();

        // Look up the transformation from base frame to camera link
        try {
            // Save the transformation from base to frame
            geometry_msgs::TransformStamped b2s = tfBuffer->lookupTransform(baseFrameId, cameraFrameId, grabTime);
            // Get the TF2 transformation
            tf2::fromMsg(b2s.transform, baseToSensor);

        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                    "will assume it as identity!",
                    baseFrameId.c_str(),
                    cameraFrameId.c_str());
            ROS_DEBUG("Transform error: %s", ex.what());
            baseToSensor.setIdentity();
        }

        if ( publishTf) {
            //Note, the frame is published, but its values will only change if someone has subscribed to odom
            publishTrackedFrame(baseTransform, transformOdomBroadcaster, baseFrameId, grabTime); //publish the tracked Frame
        }
        ros::spinOnce();
        loopRate.sleep();
    }
}

void ZedPilotNode::publishPose(sl::Pose &pose)
{
    if (odometryPub.getNumSubscribers() > 0) {
        // Transform ZED pose in TF2 Transformation
        tf2::Transform camera_transform;
        geometry_msgs::Transform c2s;
        sl::Translation translation = pose.getTranslation();
        c2s.translation.x = translation(2);
        c2s.translation.y = -translation(0);
        c2s.translation.z = -translation(1);
        sl::Orientation quat = pose.getOrientation();
        c2s.rotation.x = quat(2);
        c2s.rotation.y = -quat(0);
        c2s.rotation.z = -quat(1);
        c2s.rotation.w = quat(3);
        tf2::fromMsg(c2s, camera_transform);
        // Transformation from camera sensor to base frame
        baseTransform = baseToSensor * camera_transform * baseToSensor.inverse();
        // Publish odometry message
        publishOdom(baseTransform, odometryFrameId, grabTime);
    }
}

void ZedPilotNode::publishVelositySP(const Twist &twist)
{
    velocitySP.header.stamp = ros::Time::now();
    velocitySP.header.seq++;
    auto &l = velocitySP.twist.linear, &a = velocitySP.twist.angular;
    l.x = twist.linear[0];
    l.y = twist.linear[1];
    l.z = twist.linear[2];
    a.x = twist.angular[0];
    a.y = twist.angular[1];
    a.z = twist.angular[2];
    velocitySpPub.publish(velocitySP);
}

void ZedPilotNode::publishOdom(tf2::Transform baseTransform, string odomFrame, ros::Time t) {
    nav_msgs::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odomFrame; // odom_frame
    odom.child_frame_id = baseFrameId; // base_frame
    // conversion from Tranform to message
    geometry_msgs::Transform base2 = tf2::toMsg(baseTransform);
    // Add all value in odometry message
    odom.pose.pose.position.x = base2.translation.x;
    odom.pose.pose.position.y = base2.translation.y;
    odom.pose.pose.position.z = base2.translation.z;
    odom.pose.pose.orientation.x = base2.rotation.x;
    odom.pose.pose.orientation.y = base2.rotation.y;
    odom.pose.pose.orientation.z = base2.rotation.z;
    odom.pose.pose.orientation.w = base2.rotation.w;
    // Publish odometry message
    odometryPub.publish(odom);
}

/* \brief Publish the pose of the camera as a transformation
 * \param base_transform : Transformation representing the camera pose from base frame
 * \param trans_br : the TransformBroadcaster object to use
 * \param odometry_transform_frame_id : the id of the transformation
 * \param t : the ros::Time to stamp the image
 */
void ZedPilotNode::publishTrackedFrame(tf2::Transform baseTransform, tf2_ros::TransformBroadcaster &trans_br, string odometryTransformFrameId, ros::Time t) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odometryFrameId;
    transformStamped.child_frame_id = odometryTransformFrameId;
    // conversion from Tranform to message
    transformStamped.transform = tf2::toMsg(baseTransform);
    // Publish transformation
    trans_br.sendTransform(transformStamped);
}

void ZedPilotNode::publishState()
{
#ifdef GEM_MSG
    zed_pilot::ZedState state;
    state.header.stamp = ros::Time::now();
    state.fps = camera.getCurrentFPS();
    state.poseConfidence = (unsigned char)pose.pose_confidence;
    if (svoRecordingEnabled) {
        state.svoFrame = svoFramesRecorded;
        state.svoRecordNumber = svoRecordNumber;
    } else {
        state.svoFrame = camera.getSVOPosition();
        state.svoRecordNumber = -1;
    }
#else
    string stateStr;
    stateStr += "fps: " + to_string(camera.getCurrentFPS());
    stateStr += "\nconf:" + to_string(pose.pose_confidence);
    if (svoRecordingEnabled) {
        stateStr += "\nsvoFrame: " + to_string(svoFramesRecorded);
        stateStr += "\nsvoRecordNumber: " + to_string(svoRecordNumber);
    } else {
        stateStr += "\nsvoFrame: " + camera.getSVOPosition();
    }
    std_msgs::String state;
    state.data = stateStr;
#endif
    statePub.publish(state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pilot");
    ZedPilotNode node;
    node.init();
    node.spin();
    node.shutdown();
}
