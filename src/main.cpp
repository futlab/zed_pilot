
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <stdlib.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "wirefinder.h"
#include "controllers.h"

// ZED includes
#include <sl_zed/Camera.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/gpu

class NavSatSubscriber
{
    void callback(const sensor_msgs::NavSatFix::ConstPtr& fix) {
        lat = fix->latitude;
        lon = fix->longitude;
        if (std::isnan(hlat) || std::isnan(hlon)) {
            hlat = lat;
            hlon = lon;
        }
    }
    ros::Subscriber sub_;
public:
    double lat, lon, hlat, hlon;
    NavSatSubscriber(ros::NodeHandle &nh, const std::string& topic = "/mavros/global_position/raw/fix") : hlat(NAN), hlon(NAN) {
        sub_ = nh.subscribe<sensor_msgs::NavSatFix>(topic, 1, &NavSatSubscriber::callback, this);
    }
};


class PosePoseSubscriber
{
  public:
    PosePoseSubscriber(geometry_msgs::Pose *pose_ptr_in, const std::string& topic )
    {
        pose_ptr = pose_ptr_in;
        mav_pose_sub = nh_.subscribe<geometry_msgs::Pose>(topic, 1, &PosePoseSubscriber::MavCallback, this);
    }

  private:
    ros::NodeHandle nh_;
    geometry_msgs::Pose *pose_ptr;
    ros::Subscriber mav_pose_sub;

    void MavCallback(const geometry_msgs::Pose::ConstPtr &pose)
    {
        *pose_ptr = *pose;
    }
};


class PoseSubscriber
{
  public:
    PoseSubscriber(geometry_msgs::PoseStamped *pose_ptr_in, const std::string& topic )
    {
        pose_ptr = pose_ptr_in;
        mav_pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(topic, 1, &PoseSubscriber::MavCallback, this);
    }

  private:
    ros::NodeHandle nh_;
    geometry_msgs::PoseStamped *pose_ptr;
    ros::Subscriber mav_pose_sub;

    void MavCallback(const geometry_msgs::PoseStamped::ConstPtr &pose)
    {
        *pose_ptr = *pose;
    }
};

class MavStateSubscriber
{
  public:
    MavStateSubscriber(mavros_msgs::State *state_ptr_in)
    {
        state_ptr = state_ptr_in;
        mav_state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 1, &MavStateSubscriber::MavCallback, this);
    }

  private:
    ros::NodeHandle nh_;
    mavros_msgs::State *state_ptr;
    ros::Subscriber mav_state_sub;

    void MavCallback(const mavros_msgs::State::ConstPtr &state)
    {
        *state_ptr = *state;
    }
};

class HitDisarm
{
  public:
    HitDisarm(ros::NodeHandle nh, const std::string &imuTopic = "mavros/imu/data", const std::string &armService = "mavros/cmd/arming") {
         imuSub = nh.subscribe<sensor_msgs::Imu>(imuTopic, 100, &HitDisarm::imuCallback, this);
         armSC = nh.serviceClient<mavros_msgs::CommandBool>(armService);
    }
    void arm(bool val = true) {
        armSrv.request.value = val;
        armSC.call(armSrv);
    }
    double thresh = 25;
  private:
    ros::Subscriber imuSub;
    mavros_msgs::CommandBool armSrv;
    ros::ServiceClient armSC;
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        if(msg->linear_acceleration.z > thresh) {
            armSrv.request.value = false;
            armSC.call(armSrv);
        }
    }
};

cv::Mat slMat2cvMat(sl::Mat& input)
{
    using namespace sl;
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

void process(unsigned int *left, int leftStep, unsigned int *right, int rightStep, unsigned char *result, int resStep);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autopilot");
    ros::NodeHandle nh_, nhp("~");
    ros::Rate rate(60.0);

    sl::Camera zed;
    WireFinder wf;

    // Create configuration parameters
    sl::InitParameters initParams;
    std::string svoFileName = "/home/igor/svo/test2.svo";
    //if (nhp.getParam("max_out_fps", svoFileName)) {
        initParams.svo_input_filename = sl::String(svoFileName.c_str());
    //}

    initParams.sdk_verbose = true; // Enable the verbose mode
    initParams.depth_mode = sl::DEPTH_MODE_PERFORMANCE; // Set the depth mode to performance (fastest)
    //initParams.

    // Open the camera
    sl::ERROR_CODE err = zed.open(initParams);
    if (err != sl::SUCCESS) {
        ROS_FATAL("Unable to open ZED camera, code %d", (int)err);
        return (int)err;
    }
    sl::Mat slDepth(zed.getResolution(), sl::MAT_TYPE_32F_C1);
    cv::Mat cvDepth = slMat2cvMat(slDepth);
    sl::Mat slLeft(zed.getResolution(), sl::MAT_TYPE_8U_C4, sl::MEM_GPU), slRight(zed.getResolution(), sl::MAT_TYPE_8U_C4, sl::MEM_GPU);
    sl::Mat slMask(zed.getResolution(), sl::MAT_TYPE_8U_C1, sl::MEM_GPU);
    //cv::Mat left = slMat2cvMat(slLeft), right = slMat2cvMat(slRight);

    float min = zed.getDepthMinRangeValue(), max = zed.getDepthMaxRangeValue();
    bool pause = false, ni = false;
    int disp = 0;
    //sl::ma
    while (ros::ok()) {


        if (!pause) {
            if (zed.grab(sl::RuntimeParameters(sl::SENSING_MODE_STANDARD, true, false)) == sl::SUCCESS) {
                zed.retrieveImage(slLeft, sl::VIEW_LEFT, sl::MEM_GPU);
                zed.retrieveImage(slRight, sl::VIEW_RIGHT, sl::MEM_GPU);

                //cv::blur(left, left, cv::Size(11, 11));
                //cv::blur(right, right, cv::Size(11, 11));

                ni = true;
            }
        } else
            ni = true;
        if (ni) {
            ni = false;
            //cv::imshow("Left", left);
            //cv::imshow("Right", right);
            process( (unsigned int*)slLeft.getPtr<sl::uchar4>(sl::MEM_GPU), slLeft.getStepBytes(sl::MEM_GPU), (unsigned int*)slRight.getPtr<sl::uchar4>(sl::MEM_GPU), slRight.getStepBytes(sl::MEM_GPU), slMask.getPtr<sl::uchar1>(sl::MEM_GPU), slMask.getStepBytes(sl::MEM_GPU));
            slMask.updateCPUfromGPU();

            cv::Mat mask = slMat2cvMat(slMask);//(cv::Size(slLeft.getWidth(), slLeft.getWidth()), CV_8U);
            int x = 0;
            /*for (unsigned char * l = left.data + disp * 4, * r = right.data, * m = mask.data; m < mask.datalimit; l += 4, r += 4, m++, x++) {
                if (mask.cols - x > disp) {
                    int sum = 0;
                    for (int c = 0; c < 3; c++)
                        sum += abs(l[c] - r[c]);
                    if (sum > 255) sum = 255;
                    *m = sum;
                } else *m = 0;
                if (x == mask.cols)
                    x = 0;
            }*/
            cv::imshow("Diff", mask);

            switch(cv::waitKey(1)) {
            case 'p': pause = !pause; break;
            case '[': if (disp > 0) disp--; break;
            case ']': if (disp < 64) disp++; break;
            }

            zed.retrieveMeasure(slDepth, sl::MEASURE_DEPTH);
            wf.onDisparity(cvDepth, min, max);
        }
        ros::spinOnce();
        rate.sleep();
    }
    zed.close();
}

/*
int main(int argc, char **argv)
{
    ros::init(argc, argv, "autopilot");
    ros::NodeHandle nh_;
    ros::Rate rate(30.0);

    geometry_msgs::Pose poi;
    PosePoseSubscriber poi_subscriber(&poi,"autopilot/poi");
    ROS_INFO("poi subscriber start");

    geometry_msgs::PoseStamped pose_in;
    PoseSubscriber pose_subscriber(&pose_in,"mavros/local_position/pose");
    ROS_INFO("local position subscriber start");

    mavros_msgs::State px_state;
    MavStateSubscriber mav_subscriber(&px_state);
    ROS_INFO("px state subscriber start");

    WireFinder wf;
    NavSatSubscriber gps(nh_);

    YawController yawCtl;



    // wait for connection
    while (ros::ok() && !px_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        if (!px_state.connected)
            ROS_INFO("FCU no connect");
    }
    ROS_INFO("FCU connected");

    double roll, pitch, yaw;
    double yaw_out;

    HitDisarm hd(nh_);

    std_msgs::Bool reply_wp_out;
    ros::Publisher reply_wp_pub = nh_.advertise<std_msgs::Bool>("autopilot/reply_wp", 1);

    geometry_msgs::TwistStamped vel = {};
    vel.header.frame_id = "0";
    ros::Publisher velocity_sp = nh_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);

    reply_wp_out.data = true;
    double spy = -70, spz = 13.5;
    //double spy = -53, spz = 8.5;

    double poix, poiy, poiz;
    enum {
        TAKEOFF,
        SEARCH,
        DESCENT
    } mode = TAKEOFF;
    int pt = 0;
    int seq = 0, ctr = 0, ctr1 = 0;

    while (ros::ok())
    {
        //get actual data
        tf::Quaternion q(pose_in.pose.orientation.x, pose_in.pose.orientation.y, pose_in.pose.orientation.z, pose_in.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        if (px_state.armed && px_state.mode == "OFFBOARD")
        { 
            double gy = (gps.lat - gps.hlat) * 1E6, gx = (gps.lon - gps.hlon) * 1E6;

            if (mode != DESCENT) {
                double spzt = (mode == TAKEOFF) ? 13.5 : spz;
                double vz = (spzt - pose_in.pose.position.z);
                if(vz > 1.5) vz = 1.5;
                else if(vz < -1.) vz = -0.5;
                vel.twist.linear.z = vz;
            }

            if (mode == TAKEOFF) {
                if (pose_in.pose.position.z < 13) {
                    vel.twist.linear.x = 0;
                    vel.twist.linear.y = 0;
                } else if (fabs(gy - spy) > 15) {
                    vel.twist.linear.y = spy - gy;
                    if (fabs(vel.twist.linear.y) > 1.5) vel.twist.linear.y /= fabs(vel.twist.linear.y) / 1.5;
                } else mode = SEARCH;
            }

            if (mode == SEARCH || mode == DESCENT) {
                if (mode == SEARCH) {
                    if (gy > spy) vel.twist.linear.y = -0.3;
                    else vel.twist.linear.y = 0.3;
                }

                vel.twist.linear.x = 0;

                if(std::isnan(wf.a_)) vel.twist.angular.z = 0;
                else if (fabs(gy - spy) < 7){
                    vel.twist.angular.z = yawCtl.calcAngularZ(wf.a_);

                    double vy = -wf.b_ * 3 - pose_in.pose.orientation.x * 0.5;
                    if (vy > 0.5) vy = 0.5;
                    else if (vy < -0.5) vy = -0.5;

                    vel.twist.linear.y = vy;
                    if (fabs(vy) < 0.1 && fabs(pose_in.pose.orientation.x) < 0.05) {
                        if (ctr > 10) {
                            mode = DESCENT;
                            /*if (ctr > 50)
                                hd.thresh = 11;* /
                        }
                        ctr++;
                        vel.twist.linear.z = -0.1;
                    } else {
                        ctr = 0;
                        vel.twist.linear.z *= 0.8;
                    }
                }
            }

            if (mode == DESCENT && std::isnan(wf.a_)) {
                ctr1++;
                if (ctr1 > 100)
                    hd.arm(false);
            } else ctr1 = 0;
        }
        else
        {
            if (mode == DESCENT) {
                ros::Duration(3).sleep();
                //spz = 7;
                pt++;
                if (pt > 1) pt = 0;
                switch (pt) {
                case 0: spy = -70; spz = 10; break;
                case 1: spy = -53; spz = 8.5; break;
                }
                mode = TAKEOFF;
                hd.arm();
            } else if (px_state.mode == "OFFBOARD") {
                hd.arm();
            }
            yaw_out = yaw;
            /*if (px_state.mode == "OFFBOARD") {
                if (ctr1 < 100) ctr1++;
                else {
                    hd.arm();
                    ctr1 = 0;
                }
            }* /
        }

        //////////////////// copter_state publish  /////////////////////////////////////////
          // x - допустим перед y - допустим право   z - up/down
        //update_pose_out(pose_out, yaw_out, q);
        //local_pos_pub.publish(pose_out);
        vel.header.stamp = ros::Time::now();
        vel.header.seq = seq++;
        velocity_sp.publish(vel);
        reply_wp_pub.publish(reply_wp_out);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
*/
