#include <iostream>
#include <opencv2/imgproc.hpp>
#ifdef SHOW_RESULT
#include <opencv2/highgui.hpp>
#endif

#include "zedpilot.h"

using namespace std;

ZedPilot::ZedPilot() : stateImageSize(672, 376), recordingEnabled(false), framesRecorded(0), serialNumber(0)
{
    parameters.sdk_verbose = true;
    parameters.coordinate_units = sl::UNIT_METER;
    parameters.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;

    trackingParameters.enable_spatial_memory = true;

    runtimeParameters.enable_depth = true;
}

void ZedPilot::warn(const std::string &message) { cout << "WARNING: " << message << endl; }
void ZedPilot::info(const string &message)      { cout << "INFO: " << message << endl; }
void ZedPilot::infoOnce(const string &message)  { cout << "INFO: " << message << endl; }
void ZedPilot::debug(const string &message)     { cout << "DEBUG: " << message << endl; }
void ZedPilot::warn(sl::ERROR_CODE code)        { warn(sl::toString(code).c_str()); }
void ZedPilot::infoOnce(sl::ERROR_CODE code)    { infoOnce(sl::toString(code).c_str()); }

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
    return cv::Mat((int)input.getHeight(), (int)input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

void ZedPilot::open()
{
    sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
    while (err != sl::SUCCESS) {
        err = camera.open(parameters);
        warn(err);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    if (!svoOutputPrefix.empty())
        enableRecording();
    enableTracking();

    slLeftImage  = make_unique<sl::Mat>(camera.getResolution(), sl::MAT_TYPE_8U_C4, sl::MEM_CPU);
    slRightImage = make_unique<sl::Mat>(camera.getResolution(), sl::MAT_TYPE_8U_C4, sl::MEM_CPU);
    leftImage = slMat2cvMat(*slLeftImage);
    rightImage = slMat2cvMat(*slRightImage);
}

void ZedPilot::open(const string &svoFileName)
{
    parameters.svo_input_filename = svoFileName.c_str();
    open();
}

void ZedPilot::open(sl::RESOLUTION resolution, int zedId)
{
    parameters.camera_resolution = resolution;
    if (serialNumber == 0)
        parameters.camera_linux_id = zedId;
    else {
        bool waiting_for_camera = true;
        while (waiting_for_camera) {
            sl::DeviceProperties prop = zedFromSN();
            if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE) {
                warn("ZED SN" + to_string(serialNumber) + " not detected ! Please connect this ZED");
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            } else {
                waiting_for_camera = false;
                parameters.camera_linux_id = prop.id;
            }
        }
    }
    open();
}

void ZedPilot::reopen()
{
    camera.close();

    info("Re-opening the ZED");
    sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
    while (err != sl::SUCCESS) {
        int id = checkCameraReady();
        if (id > 0) {
            parameters.camera_linux_id = id;
            err = camera.open(); // Try to initialize the ZED
            warn(err);
        } else info("Waiting for the ZED to be re-connected");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    if (!svoOutputPrefix.empty())
        enableRecording();
    camera.enableTracking();
}

void ZedPilot::enableRecording()
{
    std::ostringstream oss;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    oss << std::put_time(&tm, "%Y_%m_%d__%H_%M_%S.svo");
    auto path = svoOutputPrefix + oss.str();

    auto err = camera.enableRecording(path.c_str(), sl::SVO_COMPRESSION_MODE_LOSSY);

    if (err != sl::SUCCESS) {
        string message = string("SVO recording initialization error: ") + sl::toString(err).c_str();
        warn(message);
        if (err == sl::ERROR_CODE_SVO_RECORDING_ERROR)
            info(" Note : This error mostly comes from a wrong path or missing writing permissions.");
    }
    else
        recordingEnabled = true;
}

sl::DeviceProperties ZedPilot::zedFromSN()
{
    sl::DeviceProperties prop;
    auto f = sl::Camera::getDeviceList();
    for (auto &it : f) {
        if (it.serial_number == serialNumber && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
            prop = it;
    }
    return prop;
}

void ZedPilot::processFrame()
{
    assert(slLeftImage && slRightImage);
    camera.retrieveImage(*slLeftImage, sl::VIEW_LEFT, sl::MEM_CPU);
    camera.retrieveImage(*slRightImage, sl::VIEW_RIGHT, sl::MEM_CPU);

    camera.getPosition(pose);
    publishPose(pose);

    processStateImage();
}

void ZedPilot::processStateImage()
{
    cv::resize(leftImage, stateImage4, stateImageSize, 0, 0, cv::INTER_NEAREST);
    cv::cvtColor(stateImage4, stateImage, cv::COLOR_RGBA2RGB);
#ifdef SHOW_RESULT
    cv::imshow("State image", stateImage);
    cv::waitKey(1);
#endif
    publishStateImage(stateImage);
}

void ZedPilot::publishStateImage(const cv::Mat &stateImage)
{
#ifdef USE_GST
    if (videoUdpTarget != "") {
        if (!transmitter) {
            string full = buildPipelineDesc(videoUdpTarget, stateImageSize, parameters.camera_fps);
            info("cmd: gst-launch-1.0 -v " + full);
            transmitter = make_unique<Pipeline>(stateImageSize, true, full, parameters.camera_fps);
        }
        transmitter->write(stateImage);
    }
#endif
}

void ZedPilot::grab()
{
    auto grabStatus = camera.grab(runtimeParameters);

    if (grabStatus != sl::ERROR_CODE::SUCCESS) { // Detect if a error occurred (for example: the zed have been disconnected) and re-initialize the ZED

        if (grabStatus == sl::ERROR_CODE_NOT_A_NEW_FRAME) {
            debug("Wait for a new image to proceed");
        } else infoOnce(grabStatus);

        this_thread::sleep_for(std::chrono::milliseconds(2));

        std::chrono::duration<double> duration = chrono::steady_clock::now() - lastGrabTime;
        if (duration.count() > 5)
            reopen();
        return;
    } else {
        lastGrabTime = chrono::steady_clock::now();
        if (recordingEnabled) {
            camera.record();
            framesRecorded++;
        }
    }

    processFrame();
}

int ZedPilot::checkCameraReady()
{
    int id = -1;
    auto f = sl::Camera::getDeviceList();
    for (auto &it : f)
        if (it.serial_number == serialNumber && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
            id = it.id;
    return id;
}

void ZedPilot::shutdown()
{
    if (recordingEnabled)
        camera.disableRecording();
    camera.close();
}
