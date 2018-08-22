#include <string>
#include <ros/ros.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sl/Camera.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>

using namespace std;

struct Action {
    char axis;
    float valueMin, valueMax, step;
};

vector<Action> actions = {
    {'X', 100, 300, 100},
    {'T', 0.1f, 0.3f, 0.1f},
    {'X', 0, 0, 0},
    {'T', 0.1f, 0.3f, 0.1f}
};

vector<Action> actionsZ = {
    {'Z', -300, -100, 100},
    {'T', 0.2f, 0.4f, 0.1f},
    {'Z', 100, 300, 100},
    {'T', 0.2f, 0.4f, 0.1f}
};

class TesterNode
{
private:
    ros::NodeHandle nh, nhp;
    std::string logPrefix;
    mavros_msgs::ManualControl controlMsg;
    ros::Publisher manualControlPub;
    ros::Subscriber mavStateSub, mavPoseSub, odomSub;
    sl::Camera camera;
    vector<Action> actions;
    size_t actionIdx;
    ros::Time waitUntil;
    enum State {
        TAKEOFF,
        ACTION,
        WAIT,
        LANDING
    } state;
    vector<float> values;
    void initValues(const vector<Action> &actions);
    bool nextValues();
    string valuesToName();
    bool armed, failed;
    fstream log;
    int logLine = 0, poseLine = 0;
    float z0, throttle0;
    void sendManualControl();
    void mavStateCallback(const mavros_msgs::State::ConstPtr &state);
    void writeLog(const sl::Translation &translation, const sl::Orientation &orientation, sl::timeStamp ts);
    bool openLog(const string &fn);
    void mavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);
    bool startTest();
    bool doTest(const sl::Translation &translation, const sl::Orientation &orientation);
    bool nextTest();
public:
    TesterNode(const vector<Action> &actions);
    bool init();
    void spin();
    void disarm();
    void advertise();
    void subscribe();
    ~TesterNode();
};

void TesterNode::initValues(const vector<Action> &actions)
{
    this->actions = actions;
    auto l = actions.size();
    values.resize(l);
    for (size_t x = 0; x < l; ++x)
        values[x] = actions[x].valueMin;
}

bool TesterNode::nextValues()
{
    for (size_t x = actions.size() - 1; signed(x) >= 0; --x) {
        if (actions[x].step == 0.0f)
            continue;
        values[x] += actions[x].step;
        if (values[x] < actions[x].valueMax + 1E-7f)
            return true;
        values[x] = actions[x].valueMin;
    }
    return false;
}

string TesterNode::valuesToName()
{
    auto l = actions.size();
    stringstream stream;
    for (size_t x = 0; x < l; ++x) {
        if (x) stream << "_";
        if (actions[x].axis == 'T')
            stream << fixed << setprecision(2);
        else
            stream << fixed << setprecision(0);
        stream << actions[x].axis << values[x];
    }
    return stream.str();
}

void TesterNode::sendManualControl()
{
    controlMsg.header.stamp = ros::Time::now();
    controlMsg.header.seq++;
    manualControlPub.publish(controlMsg);
}

TesterNode::TesterNode(const vector<Action> &actions) : nhp("~"), armed(false)
{
    initValues(actions);
    controlMsg.buttons = 0;
}

bool TesterNode::init()
{
    if (!nhp.getParam("log_prefix", logPrefix))
    {
        ROS_FATAL("Parameter log_prefix required");
        return false;
    }
    sl::InitParameters initParams;
    initParams.camera_resolution = sl::RESOLUTION_VGA;//RESOLUTION_HD720; // Use HD720 video mode (default fps: 60)
    initParams.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE; // Use a right-handed Y-up coordinate system
    initParams.coordinate_units = sl::UNIT_METER;
    initParams.camera_fps = 0;
    initParams.sdk_verbose = true;
    string svoInput;
    if (nhp.getParam("svo_input", svoInput))
        initParams.svo_input_filename = svoInput.c_str();

    auto e = camera.open(initParams);
    if (e != sl::SUCCESS) {
        ROS_FATAL("Camera open error: %s", sl::toString(e).c_str());
        return false;
    }
    sl::TrackingParameters tracking_parameters;
    e = camera.enableTracking(tracking_parameters);
    if (e != sl::SUCCESS) {
        ROS_FATAL("Camera enable tracking error: %s", sl::toString(e).c_str());
        return false;
    }

    advertise();
    subscribe();
    startTest();
    return true;
}

bool TesterNode::startTest()
{
    string logName;
    while (!openLog(logName = logPrefix + valuesToName() + ".m")) {
        if (!nextValues())
            return false;
    }
    ROS_INFO("Started log '%s'", logName.c_str());
    state = TAKEOFF;
    actionIdx = 0;
    controlMsg.x = 0;
    controlMsg.y = 0;
    controlMsg.z = 0;
    controlMsg.r = 0;
    z0 = nanf("");
    camera.resetTracking(sl::Transform());
    return true;
}

bool TesterNode::nextTest()
{
    if (log.is_open())
        log.close();
    if (!nextValues()) return false;
    return startTest();
}

bool TesterNode::doTest(const sl::Translation &translation, const sl::Orientation &orientation)
{
    switch (state) {
    case TAKEOFF:
        if (isnanf(z0))
            z0 = -translation.z;
        else {
            if (z0 + 0.1f < -translation.z) {
                state = ACTION;
                throttle0 = controlMsg.z;
                ROS_INFO("Started actions at throttle %f", throttle0);
            } else {
                controlMsg.z += 0.5f;
                if (controlMsg.z > 500.0f) {
                    ROS_FATAL("Cannot find throttle neutral level");
                    return false;
                }
            }
        }
        break;
    case ACTION:
        if (actionIdx >= actions.size()) {
            ROS_INFO("Actions completed");
            waitUntil = ros::Time::now() + ros::Duration(3.0);
            state = LANDING;
        } else {
            char axis = actions[actionIdx].axis;
            switch(axis) {
            case 'X':
                controlMsg.x = values[actionIdx];
                break;
            case 'Y':
                controlMsg.y = values[actionIdx];
                break;
            case 'Z':
                controlMsg.z = throttle0 + values[actionIdx];
                break;
            case 'R':
                controlMsg.r = values[actionIdx];
                break;
            case 'T':
                waitUntil = ros::Time::now() + ros::Duration(double(values[actionIdx]));
                state = WAIT;
                break;
            default:
                ROS_ERROR("Unknown axis '%c'", axis);
            }
            actionIdx++;
        }
        break;
    case WAIT:
        if (ros::Time::now() < waitUntil)
            break;
        state = ACTION;
        break;
    case LANDING:
        controlMsg.x = 0;
        controlMsg.y = 0;
        controlMsg.z = 50;
        controlMsg.r = 0;
        if (ros::Time::now() < waitUntil)
            break;
        ROS_INFO("Test completed");
        return nextTest();
    /*default:
        ROS_ERROR("Unknown state %d", int(state));
        state = LANDING;
        break;*/
    }
    return true;
}

void TesterNode::spin()
{
    failed = false;
    ros::Rate waitRate(1), execRate(100);
    while (!armed) {
        if (!ros::ok())
            return;
        ROS_INFO("Wait for arming...");
        ros::spinOnce();
        waitRate.sleep();
    }
    //    initTest(test);
    while(armed && ros::ok()) {
        if (!log.is_open())
            if(!startTest())
                break;
        sl::Pose zedPose;
        if (camera.grab() == sl::SUCCESS) {
            sl::TRACKING_STATE state = camera.getPosition(zedPose, sl::REFERENCE_FRAME_WORLD);
        } else {
            ROS_FATAL("Unable to grab");
            break;
        }
        auto translation = zedPose.getTranslation();
        auto orientation = zedPose.getOrientation();

        if (!doTest(translation, orientation)) break;
        sendManualControl();
        writeLog(translation, orientation, zedPose.timestamp);
        ros::spinOnce();
        execRate.sleep();
    }
    if (armed)
        disarm();
    else
        ROS_WARN("Disarmed, terminating ...");
}

void TesterNode::disarm()
{
    ROS_INFO("Try to disarm...");
    ros::ServiceClient client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool cb;
    cb.request.value = false;
    if (client.call(cb))
        ROS_INFO("/mavros/cmd/arming returned: %d", int(cb.response.success));
    else
        ROS_FATAL("Unable to call service /mavros/cmd/arming");
}

void TesterNode::advertise()
{
    std::string controlTopic = "mavros/manual_control/send";
    manualControlPub = nh.advertise<mavros_msgs::ManualControl>(controlTopic, 1);
}

void TesterNode::subscribe()
{
    mavStateSub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &TesterNode::mavStateCallback, this);
    mavPoseSub  = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &TesterNode::mavPoseCallback, this);
}

TesterNode::~TesterNode()
{
    if (log.is_open()) log.close();
}

void TesterNode::mavStateCallback(const mavros_msgs::State::ConstPtr &state)
{
    armed = state->armed;
}

void TesterNode::mavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    if (!log.is_open()) return;
    poseLine++;
    auto &o = pose->pose.orientation;
    sl::Orientation so;
    so.x = float(o.x);
    so.y = float(o.y);
    so.z = float(o.z);
    so.w = float(o.w);
    auto e = so.getRotation().getEulerAngles();
    log << "pose.data(" << poseLine << ",:) = ["
        << o.x << "," << o.y << "," << o.z << "," << o.w << ","
        << e.x << "," << e.y << "," << e.z
        << "];" << endl;
}

inline bool fileExists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

bool TesterNode::openLog(const string &fn)
{
    if (log.is_open()) log.close();
    if (fileExists(fn))
        return false;
    log.open(fn, ios::out);
    log << "zed.columns = {'timestamp','x','y','z','roll','pitch','yaw'};" << endl;
    log << "ctl.columns = {'x','y','z','r'};"  << endl;
    log << "pose.column = {'x','y','z','w','pitch','roll','yaw'};" << endl;
    return true;
}

void TesterNode::writeLog(const sl::Translation &translation, const sl::Orientation &orientation, sl::timeStamp ts)
{
    if (!log.is_open()) return;
    sl::float3 e = orientation.getRotation().getEulerAngles();
    logLine++;
    log << "zed.data(" << logLine << ",:) = ["
        << ts << ","
        << translation.x << "," << translation.y << "," << translation.z << ","
        << e.x << "," << e.y << "," << e.z
        << "];" << endl;
    log << "ctl.data(" << logLine << ",:) = ["
        << controlMsg.x << "," << controlMsg.y << "," << controlMsg.z << "," << controlMsg.r
        << "];" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tester");
    TesterNode node(actionsZ);
    if (node.init()) {
        node.spin();
        return 0;
    } else {
        return -1;
    }
}
