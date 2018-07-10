// STL includes
#include <string>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

// Local includes
#include "zed.h"
#include "pilot.h"

using namespace std;

class ZedNode
{
private:
    ros::NodeHandle nh, nhp;
    ros::Publisher odometryPub, velocitySpPub;
    ros::Subscriber mavStateSub;
    string odometryFrameId, baseFrameId, cameraFrameId;
    unique_ptr<tf2_ros::Buffer> tfBuffer;
    bool publishTf;
    tf2_ros::TransformBroadcaster transformOdomBroadcaster;
    Zed zed;
    Pilot pilot;

    void publishOdom(tf2::Transform base_transform, string odomFrame, ros::Time t);
    void publishTrackedFrame(tf2::Transform base_transform, tf2_ros::TransformBroadcaster &trans_br, string odometryTransformFrameId, ros::Time t);
    void advertise();
    void subscribe();
    void MavStateCallback(const mavros_msgs::State::ConstPtr &state);
public:
    ZedNode() : nhp("~") {}
    void init();
    void spin();
};

void ZedNode::advertise()
{
    string odometryTopic = "odom";
    odometryPub = nh.advertise<nav_msgs::Odometry>(odometryTopic, 1);
    nhp.getParam("odometry_topic", odometryTopic);
    ROS_INFO_STREAM("Advertised on topic " << odometryTopic);
    velocitySpPub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
}

void ZedNode::subscribe()
{
    mavStateSub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &ZedNode::MavStateCallback, this);
}

void ZedNode::MavStateCallback(const mavros_msgs::State::ConstPtr &state)
{
    pilot.onState(state->connected, state->armed, state->guided, state->mode);
}

void ZedNode::init()
{
    // Defaults
    int
            resolution = sl::RESOLUTION_HD720,
            quality = sl::DEPTH_MODE_PERFORMANCE,
            sensingMode = sl::SENSING_MODE_STANDARD,
            rate = 60,
            gpuId = -1,
            zedId = 0;

    string
            svoFilepath = "";

    // Get parameters

    // Set  default coordinate frames
    // If unknown left and right frames are set in the same camera coordinate frame
    nhp.param<string>("odometry_frame", odometryFrameId, "odometry_frame");
    nhp.param<string>("base_frame", baseFrameId, "base_frame");
    nhp.param<string>("camera_frame", cameraFrameId, "camera_frame");

    nhp.getParam("resolution", resolution);
    nhp.getParam("quality", quality);
    zed.parameters.depth_mode = static_cast<sl::DEPTH_MODE>(quality);
    nhp.getParam("sensing_mode", sensingMode);
    zed.runtimeParameters.sensing_mode = static_cast<sl::SENSING_MODE>(sensingMode);
    nhp.getParam("frame_rate", rate);
    zed.parameters.camera_fps = rate;
    string odometryDB;
    nhp.getParam("odometry_DB", odometryDB);
    zed.trackingParameters.area_file_path = odometryDB.c_str();
    nhp.param<bool>("publish_tf", publishTf, false);

    nhp.getParam("gpu_id", gpuId);
    zed.parameters.sdk_gpu_id = gpuId;
    nhp.getParam("zed_id", zedId);
    int tmpSn = 0;
    nhp.getParam("serial_number", tmpSn);
    if (tmpSn > 0) zed.serialNumber = tmpSn;

    if (zed.serialNumber > 0)
        ROS_INFO_STREAM("SN : " << zed.serialNumber);

    nhp.param<string>("svo_filepath", svoFilepath, string());

    if (!svoFilepath.empty())
        zed.parameters.svo_input_filename = svoFilepath.c_str();
    else {
        zed.parameters.camera_resolution = (sl::RESOLUTION)resolution;
        if (zed.serialNumber == 0)
            zed.parameters.camera_linux_id = zedId;
        else {
            bool waiting_for_camera = true;
            while (waiting_for_camera) {
                sl::DeviceProperties prop = Zed::zedFromSN(zed.serialNumber);
                if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE) {
                    std::string msg = "ZED SN" + to_string(zed.serialNumber) + " not detected ! Please connect this ZED";
                    ROS_WARN("%s", msg.c_str());
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                } else {
                    waiting_for_camera = false;
                    zed.parameters.camera_linux_id = prop.id;
                }
            }
        }
    }

    sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
    while (err != sl::SUCCESS) {
        err = zed.open();
        ROS_WARN("%s", sl::toString(err).c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    zed.enableTracking();

    tfBuffer = make_unique<tf2_ros::Buffer>();
    advertise();
    subscribe();
}

void ZedNode::spin()
{
    ros::Time oldT;
    sl::Pose pose;
    tf2::Transform baseTransform;
    ros::Rate loopRate(zed.parameters.camera_fps);
    while(nhp.ok()) {
        ros::Time t = ros::Time::now(); // Get current time

        auto grabStatus = zed.grab();

        if (grabStatus != sl::ERROR_CODE::SUCCESS) { // Detect if a error occurred (for example: the zed have been disconnected) and re-initialize the ZED

            if (grabStatus == sl::ERROR_CODE_NOT_A_NEW_FRAME) {
                ROS_DEBUG("Wait for a new image to proceed");
            } else ROS_INFO_ONCE("%s", sl::toString(grabStatus).c_str());

            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            if ((t - oldT).toSec() > 5) {
                zed.camera.close();

                ROS_INFO("Re-opening the ZED");
                sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
                while (err != sl::SUCCESS) {
                    int id = zed.checkCameraReady();
                    if (id > 0) {
                        zed.parameters.camera_linux_id = id;
                        err = zed.open(); // Try to initialize the ZED
                        ROS_INFO_STREAM(toString(err));
                    } else ROS_INFO("Waiting for the ZED to be re-connected");
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }
                zed.enableTracking();
            }
            continue;
        } else
            oldT = ros::Time::now();

        // Transform from base to sensor
        tf2::Transform baseToSensor;
        // Look up the transformation from base frame to camera link
        try {
            // Save the transformation from base to frame
            geometry_msgs::TransformStamped b2s = tfBuffer->lookupTransform(baseFrameId, cameraFrameId, t);
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

        if (odometryPub.getNumSubscribers() > 0) {
            zed.camera.getPosition(pose);
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
            publishOdom(baseTransform, odometryFrameId, t);
        }

        if ( publishTf) {
            //Note, the frame is published, but its values will only change if someone has subscribed to odom
            publishTrackedFrame(baseTransform, transformOdomBroadcaster, baseFrameId, t); //publish the tracked Frame
        }
        ros::spinOnce();
        loopRate.sleep();
    }
}

void ZedNode::publishOdom(tf2::Transform baseTransform, string odomFrame, ros::Time t) {
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
void ZedNode::publishTrackedFrame(tf2::Transform baseTransform, tf2_ros::TransformBroadcaster &trans_br, string odometryTransformFrameId, ros::Time t) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odometryFrameId;
    transformStamped.child_frame_id = odometryTransformFrameId;
    // conversion from Tranform to message
    transformStamped.transform = tf2::toMsg(baseTransform);
    // Publish transformation
    trans_br.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pilot");
    ZedNode node;
    node.init();
    node.spin();
}
