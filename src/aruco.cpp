#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "../include/aruco.h"
#include <iostream>

using namespace Eigen;
using namespace Sophus;

struct Marker
{
	SE3d pose;
    double size;
    Marker(const cv::Vec3d &leftR, const cv::Vec3d &leftT, const cv::Vec3d &rightR, const cv::Vec3d &rightT, double baseline, double defSize)
    {
        double zlzr = leftT[2] / rightT[2];
        double lk = baseline / (leftT[0] - rightT[0] * zlzr);
        double rk = lk * zlzr;
        Vector3d p = (Map<const Vector3d>(leftT.val) * lk + Map<const Vector3d>(rightT.val) * rk) * 0.5;
        cv::Mat lr, rr;
        cv::Rodrigues(leftR, lr);
        cv::Rodrigues(rightR, rr);
		Quaterniond q, ql, qr;
		ql = Map<const Matrix3d>((double *)lr.data);
		qr = Map<const Matrix3d>((double *)rr.data);
		q = ql.slerp(0.5, qr);
        size = (lk + rk) * 0.5 * defSize;
		pose = SE3d(q, p);
    }
    Marker(){}
};

struct Frame
{
	SE3d pose;
    std::map<int, Marker> markers;
};

class Aruco_
{
private:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::map<int, Marker> markers;
    std::vector<Frame> frames;
    cv::Mat gray;
    const Aruco *const parent;
    SE3d cameraPose;
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
		Frame *frame;
		if (frames.size() < 5) {
			frames.emplace_back();
			frame = &frames.back();
		} else {
			frame = &frames[rand() % 5];
		}
        createFrame(left, right, *frame);
        if (frame->markers.empty()) {
            frames.pop_back();
            return false;
        }
        if (markers.empty())
            initializeByFrame(*frame);
        else {
            frame->pose = cameraPose;
			calculateErrors();
		}
        cameraPose = frame->pose;
#ifdef WITH_GUI
		cv::Mat leftRGB, rightRGB;
		cvtColor(left, leftRGB, cv::COLOR_BGRA2BGR);
		cvtColor(right, rightRGB, cv::COLOR_BGRA2BGR);
		drawMarkers(leftRGB, false);
		drawMarkers(rightRGB, true);
		cv::imshow("Left", leftRGB);
		cv::imshow("Right", rightRGB);
		cv::waitKey(1);
#endif
        return true;
    }
	void drawMarkers(cv::Mat &out, bool right)
	{
		auto icp = cameraPose.inverse();
		for (const auto &m : markers) {
			auto mp = icp * m.second.pose;
			cv::Vec3d tvec, rvec;
			Map<Vector3d>(tvec.val) = mp.translation() + Vector3d::UnitX() * (parent->baseline * (right ? -0.5 : 0.5));
			cv::Mat rm(3, 3, CV_64F);
			Map<Matrix3d>((double *)rm.data) = mp.rotationMatrix();
			cv::Rodrigues(rm, rvec);
			cv::aruco::drawAxis(out, parent->cameraMatrix, parent->distCoeffs, rvec, tvec, m.second.size);
		}
	}
private:
    void imageToMarkers(const cv::Mat &image, std::vector<int> &markerIds, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs)
    {
        using namespace cv;
        cv::cvtColor(image, gray, COLOR_BGRA2GRAY);
        GaussianBlur(gray, gray, Size(7, 7), 0);
        aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        aruco::estimatePoseSingleMarkers(markerCorners, float(markerDefSize), parent->cameraMatrix, parent->distCoeffs, rvecs, tvecs);

        /*Mat rgb;
        cvtColor(image, rgb, COLOR_BGRA2BGR);
        aruco::drawDetectedMarkers(rgb, markerCorners, markerIds);
        for(size_t i=0; i < markerIds.size(); i++)
                cv::aruco::drawAxis(rgb, parent->cameraMatrix, parent->distCoeffs, rvecs[i], tvecs[i], 0.05);
        imshow("Markers", rgb);
        waitKey();*/
    }
    void initializeByFrame(Frame &frame)
    {
        assert(!frame.markers.empty());
        const SE3d &pose = frame.markers.begin()->second.pose;
		frame.pose = pose.inverse();
        for (auto p : frame.markers) {
			p.second.pose = frame.pose * p.second.pose;
            //p.second.p = rm * p.second.p + frame.p;
            //p.second.q = frame.q * p.second.q;
            auto ir = markers.insert(p);
			if (!ir.second)
				ir.first->second = p.second;
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
		out.markers.clear();
        for (size_t ir = 0; ir < rightIds.size(); ++ir) {
            auto it = ids.find(rightIds[ir]);
            if (it != ids.end()) {
                size_t il = it->second;
                out.markers.insert(make_pair(rightIds[ir], Marker(leftR[il], leftT[il], rightR[ir], rightT[ir], parent->baseline, markerDefSize)));
            }
        }
    }
    void calculateErrors()
    {
		using namespace std;
		MatrixXd J;
		VectorXd e;
		J.resize(
			frames.size() * markers.size() * 6, // Equation count
			(frames.size() + markers.size()) * 6 // Variables count
			);
		J.setZero();
		e.resize(J.rows());
		//std::cout << "calcErrors:" << std::endl;
		for (int i = 1; i < 4; i++) {
			int row = 0, frameCol = 0;
			for (auto &f : frames) {
				int markerCol = frames.size() * 6;
				auto rm = f.pose.rotationMatrix();// f.q.toRotationMatrix();
				for (auto &m : f.markers) {
					//std::cout << "marker " << m.first << std::endl;
					auto &gm = markers.at(m.first);
					e.template block<6, 1>(row, 0) = (gm.pose * (f.pose * m.second.pose).inverse()).log();
					//                                6*var*M   6*var*F       const   
					J.template block<6, 6>(row, markerCol) = Matrix6d::Identity();
					J.template block<6, 6>(row, frameCol)  = -Matrix6d::Identity();
					markerCol += 6;
					row += 6;

					/*auto ep = gm.pose.translation() - rm * m.second.pose.translation() - f.pose.translation();
					auto eq = gm.pose.so3().unit_quaternion().coeffs() - (f.pose.so3() * m.second.pose.so3()).unit_quaternion().coeffs();
					std::cout << "e:" << e << std::endl;
					std::cout << "eq:" << eq << std::endl;
					std::cout << "ep:" << ep << std::endl;*/
				}
				frameCol += 6;
			}
			std::cout << "Iteration " << i << " error " << e.squaredNorm() << std::endl;
			//cout << "J =" << endl << J << endl;
			//cout << "J^-1 =" << endl << J.inverse() << endl;
			auto a = J.transpose() * J;
			auto b = J.transpose() * -e;
			VectorXd dT = a.ldlt().solve(b);
			//cout << "dT =" << endl << dT << endl;
			row = 0;
			for (auto &f : frames) {
				Vector6d tg = dT.template block<6, 1>(row, 0);
				f.pose = SE3d::exp(tg) * f.pose;
				row += 6;
			}
			for (auto &m : markers) {
				m.second.pose = SE3d::exp(dT.template block<6, 1>(row, 0)) * m.second.pose;
				row += 6;
			}
		}
    }
	void optimize()
	{

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


#ifdef WITH_GUI
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
    return aruco_->processFrame(left, right);
}

Aruco::~Aruco()
{
    delete aruco_;
}

