#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "../include/aruco.h"
#include <iostream>

using namespace Eigen;


struct Pose
{
    Quaterniond q;
    Vector3d p;
    double size;
    Pose(const cv::Vec3d &leftR, const cv::Vec3d &leftT, const cv::Vec3d &rightR, const cv::Vec3d &rightT, double baseline, double defSize)
    {
        double zlzr = leftT[2] / rightT[2];
        double lk = baseline / (leftT[0] - rightT[0] * zlzr);
        double rk = lk * zlzr;
        p = (Map<const Vector3d>(leftT.val) * lk + Map<const Vector3d>(rightT.val) * rk) * 0.5;
        cv::Mat lr, rr;
        cv::Rodrigues(leftR, lr);
        cv::Rodrigues(rightR, rr);
        q = (Map<const Matrix3d>((double *)lr.data) + Map<const Matrix3d>((double *)rr.data));
        q.normalize();
        size = (lk + rk) * 0.5 * defSize;
    }
    Pose(){}
};

struct Frame : public Pose
{
    std::map<int, Pose> markers;
};

class Aruco_
{
private:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::map<int, Pose> markers;
    std::vector<Frame> frames;
    cv::Mat gray;
    const Aruco *const parent;
    Pose cameraPose;
    double markerDefSize;
public:
    Aruco_(const Aruco *parent) :
        dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)),
        parameters(cv::aruco::DetectorParameters::create()),
        parent(parent),
        markerDefSize(0.05)
    {
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;//CORNER_REFINE_SUBPIX;
        parameters->cornerRefinementMinAccuracy = 0.05;
    }
    bool processFrame(const cv::Mat &left, const cv::Mat &right)
    {
        frames.emplace_back();
        auto &frame = frames.back();
        createFrame(left, right, frame);
        if (frame.markers.empty()) {
            frames.pop_back();
            return false;
        }
        if (markers.empty())
            initializeByFrame(frame);
        else {
            frame.p = cameraPose.p;
            frame.q = cameraPose.q;
        }

        calculateErrors();

        cameraPose.p = frame.p;
        cameraPose.q = frame.q;
        return true;
    }
private:
    void imageToMarkers(const cv::Mat &image, std::vector<int> &markerIds, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs)
    {
        using namespace cv;
        cvtColor(image, gray, COLOR_BGRA2GRAY);
        GaussianBlur(gray, gray, Size(7, 7), 0);
        aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        aruco::estimatePoseSingleMarkers(markerCorners, float(markerDefSize), parent->cameraMatrix, parent->distCoeffs, rvecs, tvecs);
    }
    void initializeByFrame(Frame &frame)
    {
        assert(!frame.markers.empty());
        const Pose &pose = frame.markers.begin()->second;
        frame.q = pose.q.conjugate();
        auto rm = frame.q.toRotationMatrix();
        frame.p = rm * -pose.p;
        Pose pose1 = frame.markers.begin()->second;
        for (auto p : frame.markers) {
            p.second.p = rm * p.second.p + frame.p;
            p.second.q = frame.q * p.second.q;
            markers.insert(p);
        }
    }
    void createFrame(const cv::Mat &left, const cv::Mat &right, Frame &out)
    {
        using namespace std;
        vector<int> leftIds, rightIds;
        vector<cv::Vec3d> leftR, leftT, rightR, rightT;
        imageToMarkers(left, leftIds, leftR, leftT);
        imageToMarkers(right, rightIds, rightR, rightT);
        map<int, size_t> ids;
        for (size_t il = 0; il < leftIds.size(); ++il)
            ids.insert(make_pair(leftIds[il], il));
        for (size_t ir = 0; ir < rightIds.size(); ++ir) {
            auto it = ids.find(rightIds[ir]);
            if (it != ids.end()) {
                size_t il = it->second;
                out.markers.insert(make_pair(rightIds[ir], Pose(leftR[il], leftT[il], rightR[ir], rightT[ir], parent->baseline, markerDefSize)));
            }
        }
    }
    void calculateErrors()
    {

        for (auto &f : frames) {
            auto rm = f.q.toRotationMatrix();
            for (auto &m : f.markers) {
                auto &gm = markers.at(m.first);
                auto ep = gm.p - rm * m.second.p - f.p;
                auto eq = gm.q.coeffs() - (f.q * m.second.q).coeffs();
                //std::cout << "eq:" << eq << std::endl;
                //std::cout << "ep:" << ep << std::endl;
            }
        }
    }
};


/*bool Aruco::process(const cv::Mat &left, const cv::Mat &right);
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Mat gray, rgb;
    cv::cvtColor(image, rgb, cv::COLOR_BGRA2BGR);
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(7, 7), 0);
    cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);


#ifdef SHOW_RESULT
    cv::aruco::drawDetectedMarkers(rgb, markerCorners, markerIds);

    for(int i=0; i < markerIds.size(); i++)
            cv::aruco::drawAxis(rgb, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);

    cv::imshow("Left", rgb);
    cv::waitKey(1);
#endif
    return true;
}*/


/*** Aruco methods ***/

Aruco::Aruco() :
    aruco_(new Aruco_(this)),
    cameraMatrix(3, 3, CV_64F, cv::Scalar(0)),
    distCoeffs(1, 4, CV_64F, cv::Scalar(0))
{
    cameraMatrix.at<double>(2, 2) = 1.0;
}

bool Aruco::process(const cv::Mat &left, const cv::Mat &right)
{
    aruco_->processFrame(left, right);
}

Aruco::~Aruco()
{
    delete aruco_;
}

