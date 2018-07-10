// STL includes
#include <string>

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

// ZED includes
#include "zed.h"

using namespace std;

class ZedNode
{
private:
    ros::NodeHandle nh, nhp;
    string odometryFrame, baseFrame, cameraFrame;
    Zed zed;

    void publishOdom(tf2::Transform base_transform, ros::Publisher &pub_odom, string odomFrame, ros::Time t);
    void publishTrackedFrame(tf2::Transform base_transform, tf2_ros::TransformBroadcaster &trans_br, string odometryTransformFrameId, ros::Time t);
public:
    ZedNode() : nhp("~") {}
    void init();
    void spin();
};

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

    unsigned int
            serialNumber = 0;

    string
            odometryTopic = "odom",
            svoFilepath = "";

    // Get parameters

    // Set  default coordinate frames
    // If unknown left and right frames are set in the same camera coordinate frame
    nhp.param<string>("odometry_frame", odometryFrame, "odometry_frame");
    nhp.param<string>("base_frame", baseFrame, "base_frame");
    nhp.param<string>("camera_frame", cameraFrame, "camera_frame");

    nhp.getParam("resolution", resolution);
    nhp.getParam("quality", quality);
    zed.parameters.depth_mode = static_cast<sl::DEPTH_MODE>(quality);
    nhp.getParam("sensing_mode", sensingMode);
    nhp.getParam("frame_rate", rate);
    string odometryDB;
    nhp.getParam("odometry_DB", odometryDB);
    zed.trackingParameters.area_file_path = odometryDB.c_str();
    nhp.getParam("gpu_id", gpuId);
    zed.parameters.sdk_gpu_id = gpuId;
    nhp.getParam("zed_id", zedId);
    int tmpSn = 0;
    nhp.getParam("serial_number", tmpSn);
    if (tmpSn > 0) serialNumber = tmpSn;

    if (serialNumber > 0)
        ROS_INFO_STREAM("SN : " << serialNumber);

    nhp.getParam("odometry_topic", odometryTopic);
    nhp.param<string>("svo_filepath", svoFilepath, string());

    if (!svoFilepath.empty())
        zed.parameters.svo_input_filename = svoFilepath.c_str();
    else {
        zed.parameters.camera_fps = rate;
        zed.parameters.camera_resolution = (sl::RESOLUTION)resolution;
        if (serialNumber == 0)
            zed.parameters.camera_linux_id = zedId;
        else {
            bool waiting_for_camera = true;
            while (waiting_for_camera) {
                sl::DeviceProperties prop = Zed::zedFromSN(serialNumber);
                if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE) {
                    std::string msg = "ZED SN" + to_string(serialNumber) + " not detected ! Please connect this ZED";
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
        ROS_WARN("%s", toString(err).c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
}


void ZedNode::spin()
{
    while(nhp.ok()) {

    }
}

void ZedNode::publishOdom(tf2::Transform base_transform, ros::Publisher &pub_odom, string odomFrame, ros::Time t) {
    nav_msgs::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odomFrame; // odom_frame
    odom.child_frame_id = baseFrame; // base_frame
    // conversion from Tranform to message
    geometry_msgs::Transform base2 = tf2::toMsg(base_transform);
    // Add all value in odometry message
    odom.pose.pose.position.x = base2.translation.x;
    odom.pose.pose.position.y = base2.translation.y;
    odom.pose.pose.position.z = base2.translation.z;
    odom.pose.pose.orientation.x = base2.rotation.x;
    odom.pose.pose.orientation.y = base2.rotation.y;
    odom.pose.pose.orientation.z = base2.rotation.z;
    odom.pose.pose.orientation.w = base2.rotation.w;
    // Publish odometry message
    pub_odom.publish(odom);
}

/* \brief Publish the pose of the camera as a transformation
 * \param base_transform : Transformation representing the camera pose from base frame
 * \param trans_br : the TransformBroadcaster object to use
 * \param odometry_transform_frame_id : the id of the transformation
 * \param t : the ros::Time to stamp the image
 */
void ZedNode::publishTrackedFrame(tf2::Transform base_transform, tf2_ros::TransformBroadcaster &trans_br, string odometryTransformFrameId, ros::Time t) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odometryFrame;
    transformStamped.child_frame_id = odometryTransformFrameId;
    // conversion from Tranform to message
    transformStamped.transform = tf2::toMsg(base_transform);
    // Publish transformation
    trans_br.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pilot");
    ZedNode node;
    node.spin();


}
