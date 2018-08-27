#include <string>
#include <chrono>
#include <thread>
#include <sl/Camera.hpp>
#include <sys/stat.h>
#include <fstream>
#include "aruco.h"
#ifdef WITH_GUI
#include <opencv2/highgui.hpp>
#endif

#ifdef WITH_ROS
#include <ros/ros.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#else
#define ROS_FATAL printf
#define ROS_INFO printf
#define ROS_ERROR printf
#endif

#ifdef __unix__
#include <unistd.h>
inline bool fileExists(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}
#endif

#ifdef _WIN32
#include <Shlwapi.h>
inline bool fileExists(const std::string& name) {
	return PathFileExists(name.c_str());
}
#endif

using namespace std;

struct TestAction {
    char axis;
    float valueMin, valueMax, step;
};

vector<TestAction> actions = {
    {'X', 100, 300, 100},
    {'T', 0.1f, 0.3f, 0.1f},
    {'X', 0, 0, 0},
    {'T', 0.1f, 0.3f, 0.1f}
};

vector<TestAction> actionsZ = {
    {'Z', -300, -100, 100},
    {'T', 0.2f, 0.4f, 0.1f},
    {'Z', 100, 300, 100},
    {'T', 0.2f, 0.4f, 0.1f}
};

class Camera : public sl::Camera
{
private:
	std::unique_ptr<sl::Mat> slLeftImage, slRightImage;
	cv::Mat leftImage, rightImage;
	Aruco aruco;
	static cv::Mat slMat2cvMat(sl::Mat& input)
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
		return cv::Mat(int(input.getHeight()), int(input.getWidth()), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
	}
public:
	sl::Pose zedPose;
	bool init(const string &svoInput)
	{
		sl::InitParameters initParams;
		initParams.camera_resolution = sl::RESOLUTION_VGA;//RESOLUTION_HD720; // Use HD720 video mode (default fps: 60)
		initParams.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE; // Use a right-handed Y-up coordinate system
		initParams.coordinate_units = sl::UNIT_METER;
		initParams.camera_fps = 0;
		initParams.sdk_verbose = true;
		initParams.svo_input_filename = svoInput.c_str();

		auto e = open(initParams);
		if (e != sl::SUCCESS) {
			ROS_FATAL("Camera open error: %s", sl::toString(e).c_str());
			return false;
		}
		sl::TrackingParameters tracking_parameters;
		e = enableTracking(tracking_parameters);
		if (e != sl::SUCCESS) {
			ROS_FATAL("Camera enable tracking error: %s", sl::toString(e).c_str());
			return false;
		}
		slLeftImage = make_unique<sl::Mat>(getResolution(), sl::MAT_TYPE_8U_C4, sl::MEM_CPU);
		slRightImage = make_unique<sl::Mat>(getResolution(), sl::MAT_TYPE_8U_C4, sl::MEM_CPU);
		leftImage = slMat2cvMat(*slLeftImage);
		rightImage = slMat2cvMat(*slRightImage);
		auto ci = getCameraInformation();
		auto cam = ci.calibration_parameters.left_cam;
        aruco.cameraMatrix.at<float>(0, 0) = cam.fx;
        aruco.cameraMatrix.at<float>(1, 1) = cam.fy;
        aruco.cameraMatrix.at<float>(0, 2) = cam.cx;
        aruco.cameraMatrix.at<float>(1, 2) = cam.cy;
		aruco.baseline = double(ci.calibration_parameters.T.x);
		return true;
	}
	bool processImage()
	{
		auto e = grab();
		if (e == sl::SUCCESS) {
			sl::TRACKING_STATE state = getPosition(zedPose, sl::REFERENCE_FRAME_WORLD);
        } else {
			if (e == sl::ERROR_CODE_NOT_A_NEW_FRAME) {
				return true;
			}
			ROS_FATAL("Unable to grab: %s", sl::toString(e).c_str());
			return false;
		}
		retrieveImage(*slLeftImage, sl::VIEW_LEFT, sl::MEM_CPU);
		retrieveImage(*slRightImage, sl::VIEW_RIGHT, sl::MEM_CPU);

		aruco.process(leftImage, rightImage);
#ifdef WITH_GUI
#endif
		return true;
	}

};

class Tester;

class LogWriter
{
private:
    int line, valCount;
    const int fieldCount;
    fstream *log;
    friend class Tester;
    string name, header;

    void init(fstream *log)
    {
        this->log = log;
        line = 1;
        valCount = 0;
        *log << header;
    }
public:
    LogWriter(const string &name, const vector<string> &fields) : line(1), fieldCount(int(fields.size())), name(name)
    {
        header = name + ".columns = {";
        for (auto &f : fields)
            header += "'" + f + "', ";
        header.resize(header.size() - 2);
        header += "};\n";
    }
    template<typename T>
    LogWriter& operator<<(const T& value)
    {
        if (value == endl) {
            assert(fieldCount == valCount);
            valCount = 0;
            line++;
            *log << "];" << endl;
        } else {
            if (!valCount)
                *log << name << ".data(" << line << ",:) = [";
            else
                *log << ",";
            valCount++;
            assert(valCount <= fieldCount);
            *log << value;
        }
        return *this;
    }
};

class Tester
{
private:
	vector<TestAction> actions;
	size_t actionIdx;
	enum State {
		TAKEOFF,
		ACTION,
		WAIT,
		LANDING
	} state;
	vector<float> values;
    struct Control { float x, y, z, r; } control;
	fstream log;
    chrono::time_point<chrono::high_resolution_clock> waitUntil;
	int logLine = 0, poseLine = 0;
public:
    bool armed, failed, armCheck;
    float z0, throttle0;
    vector<LogWriter *> writers;
    LogWriter controlWriter, zedWriter;
    inline bool isOpen() const { return log.is_open(); }
    std::string logPrefix;
    Tester() :
        control{},
        armed(false), armCheck(true),
        controlWriter("ctl", {"x", "y", "z", "r"}),
        zedWriter("zed", {"timestamp", "x", "y", "z", "roll", "pitch", "yaw"})
    {
    }
    bool openLog(const string &fn)
	{
		if (log.is_open()) log.close();
		if (fileExists(fn))
			return false;
		log.open(fn, ios::out);
        for (auto &w : writers)
            w->init(&log);

        /*log << "zed.columns = {'timestamp','x','y','z','roll','pitch','yaw'};" << endl;
		log << "ctl.columns = {'x','y','z','r'};" << endl;
        log << "pose.column = {'x','y','z','w','pitch','roll','yaw'};" << endl;*/
		return true;
	}
	void initValues(const vector<TestAction> &actions)
	{
		this->actions = actions;
		auto l = actions.size();
		values.resize(l);
		for (size_t x = 0; x < l; ++x)
			values[x] = actions[x].valueMin;
	}
	bool nextValues()
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
    string valuesToName() const
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
	void writeLog(const sl::Translation &translation, const sl::Orientation &orientation, sl::timeStamp ts)
	{
		if (!log.is_open()) return;
		sl::float3 e = orientation.getRotation().getEulerAngles();
		logLine++;
		log << "zed.data(" << logLine << ",:) = ["
			<< ts << ","
			<< translation.x << "," << translation.y << "," << translation.z << ","
			<< e.x << "," << e.y << "," << e.z
			<< "];" << endl;
		/*log << "ctl.data(" << logLine << ",:) = ["
			<< controlMsg.x << "," << controlMsg.y << "," << controlMsg.z << "," << controlMsg.r
			<< "];" << endl;*/
	}
	bool startTest()
	{
		string logName;
		while (!openLog(logName = logPrefix + valuesToName() + ".m")) {
			if (!nextValues())
				return false;
		}
		ROS_INFO("Started log '%s'", logName.c_str());
		state = TAKEOFF;
		actionIdx = 0;
        z0 = nanf("");
		return true;
	}
	bool nextTest()
	{
		if (log.is_open())
			log.close();
		if (!nextValues()) return false;
		return startTest();
	}
    bool doTest(const sl::Translation &translation, const sl::Orientation &orientation)
    {
        switch (state) {
        case TAKEOFF:
            if (isnanf(z0))
                z0 = -translation.z;
            else {
                if (z0 + 0.1f < -translation.z) {
                    state = ACTION;
                    throttle0 = control.z;
                    ROS_INFO("Started actions at throttle %f", throttle0);
                } else {
                    control.z += 0.5f;
                    if (control.z > 500.0f) {
                        ROS_FATAL("Cannot find throttle neutral level");
                        return false;
                    }
                }
            }
            break;
        case ACTION:
            if (actionIdx >= actions.size()) {
                ROS_INFO("Actions completed");
                waitUntil = chrono::high_resolution_clock::now() + chrono::seconds(3);
                state = LANDING;
            } else {
                char axis = actions[actionIdx].axis;
                switch(axis) {
                case 'X':
                    control.x = values[actionIdx];
                    break;
                case 'Y':
                    control.y = values[actionIdx];
                    break;
                case 'Z':
                    control.z = throttle0 + values[actionIdx];
                    break;
                case 'R':
                    control.r = values[actionIdx];
                    break;
                case 'T':
                    waitUntil = chrono::high_resolution_clock::now() + chrono::milliseconds(int(1000 * values[actionIdx]));
                    state = WAIT;
                    break;
                default:
                    ROS_ERROR("Unknown axis '%c'", axis);
                }
                actionIdx++;
            }
            break;
        case WAIT:
            if (chrono::high_resolution_clock::now() < waitUntil)
                break;
            state = ACTION;
            break;
        case LANDING:
            control.x = 0;
            control.y = 0;
            control.z = 50;
            control.r = 0;
            if (chrono::high_resolution_clock::now() < waitUntil)
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
    ~Tester() { if (log.is_open()) log.close(); }
};

#ifdef WITH_ROS
class TesterNode
{
private:
	Tester tester;
    Camera camera;
    LogWriter poseWriter;
    ros::NodeHandle nh, nhp;
    mavros_msgs::ManualControl controlMsg;
    ros::Publisher manualControlPub;
    ros::Subscriber mavStateSub, mavPoseSub, odomSub;
	ros::Time waitUntil;
	void mavStateCallback(const mavros_msgs::State::ConstPtr &state);
	void mavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose);

	void sendManualControl()
	{
		controlMsg.header.stamp = ros::Time::now();
		controlMsg.header.seq++;
		manualControlPub.publish(controlMsg);
	}
    bool startTest();
    bool doTest(const sl::Translation &translation, const sl::Orientation &orientation);
    bool nextTest();
    void processImage();
public:
    TesterNode(const vector<TestAction> &actions) : nhp("~"),
        poseWriter("pose", {"x", "y", "z", "w", "pitch", "roll", "yaw"})
	{
		tester.initValues(actions);
        tester.writers.push_back(&poseWriter);
		controlMsg.buttons = 0;
	}
	bool init()
	{
        if (!nhp.getParam("log_prefix", tester.logPrefix))
		{
			ROS_FATAL("Parameter log_prefix required");
			return false;
		}
        if (nhp.getParam("arm_check", tester.armCheck)) {
            tester.armed = !tester.armCheck;
	}

		string svoInput;
		nhp.getParam("svo_input", svoInput);


		advertise();
		subscribe();

		return true;
	}
	void spin();
    void disarm();
    void advertise();
    void subscribe();
    ~TesterNode();
};


bool TesterNode::startTest()
{
	if (!tester.startTest()) return false;
    controlMsg.x = 0;
    controlMsg.y = 0;
    controlMsg.z = 0;
    controlMsg.r = 0;
    camera.resetTracking(sl::Transform());
    return true;
}

void TesterNode::spin()
{
    tester.failed = false;
	ros::Rate waitRate(1), execRate(100);
    while (!tester.armed) {
		if (!ros::ok())
			return;
		ROS_INFO("Wait for arming...");
		ros::spinOnce();
		waitRate.sleep();
	}
    while (tester.armed && ros::ok()) {
        if (!tester.isOpen())
			if (!startTest())
				break;
        if (!camera.processImage())
            break;
        auto translation = camera.zedPose.getTranslation();
        auto orientation = camera.zedPose.getOrientation();

		if (!doTest(translation, orientation)) break;
		sendManualControl();
        writeLog(translation, orientation, camera.zedPose.timestamp);
		ros::spinOnce();
		execRate.sleep();
	}
    if (tester.armed)
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
	mavPoseSub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &TesterNode::mavPoseCallback, this);
}

void TesterNode::mavStateCallback(const mavros_msgs::State::ConstPtr &state)
{
    if (tester.armCheck)
        tester.armed = state->armed;
}

void TesterNode::mavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    if (!tester.isOpen()) return;
	auto &o = pose->pose.orientation;
	sl::Orientation so;
	so.x = float(o.x);
	so.y = float(o.y);
	so.z = float(o.z);
	so.w = float(o.w);
	auto e = so.getRotation().getEulerAngles();
    /*log << "pose.data(" << poseLine << ",:) = ["
		<< o.x << "," << o.y << "," << o.z << "," << o.w << ","
		<< e.x << "," << e.y << "," << e.z
        << "];" << endl;*/
    poseWriter << o.x << o.y << o.z << o.w << e.x << e.y << e.z << endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tester");
	TesterNode node(actionsZ);
	if (node.init()) {
		node.spin();
		return 0;
	}
	else {
		return -1;
	}
}

#else

int main(int argc, char **argv)
{
	Camera camera;
	camera.init(argc > 1 ? argv[1] : "");
	while (camera.processImage())
		this_thread::sleep_for(chrono::milliseconds(1000 / 100));
	return 0;
}

#endif
