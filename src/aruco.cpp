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

using namespace std;
using namespace Eigen;
using namespace Sophus;

struct FrameMarker
{
	Vector2d leftPoints[4], rightPoints[4];
	FrameMarker(const std::vector<cv::Point2f>& leftPoints, const std::vector<cv::Point2f>& rightPoints, const cv::Mat& cameraMatrix)
	{
		assert(leftPoints.size() == 4 && rightPoints.size() == 4);
		double
			fx = cameraMatrix.at<double>(0, 0),
			fy = cameraMatrix.at<double>(1, 1),
			cx = cameraMatrix.at<double>(0, 2),
			cy = cameraMatrix.at<double>(1, 2);

		for (size_t i = 0; i < 4; i++) {
			this->leftPoints[i].x() = (double(leftPoints[i].x) - cx) / fx;
			this->leftPoints[i].y() = (double(leftPoints[i].y) - cy) / fy;
		}
		for (size_t i = 0; i < 4; i++) {
			this->rightPoints[i].x() = (double(rightPoints[i].x) - cx) / fx;
			this->rightPoints[i].y() = (double(rightPoints[i].y) - cy) / fy;
		}
	}
};

struct Frame
{
	SE3d pose, prevPose;
    std::map<int, FrameMarker> markers;
};

struct Marker
{
	Vector3d points[4], prevPoints[4];
	inline bool isMovable() { return true; }
	static void convertPoints(const Vector2d *leftPoints, const Vector2d *rightPoints, Vector3d *points, double baseline)
	{
		for (size_t i = 0; i < 4; ++i) {
			double lx = leftPoints[i].x(), rx = rightPoints[i].x(), my = (leftPoints[i].y() + rightPoints[i].y()) * 0.5;
			double z = baseline / (lx - rx);
			double x = lx * z - baseline * 0.5;
			double y = my * z;
			points[i] << x, y, z;
		}
	}
	Marker(const FrameMarker &fm, const Matrix3d &rotation, const Vector3d &translation, double baseline)
	{
		convertPoints(fm.leftPoints, fm.rightPoints, points, baseline);
		for (size_t i = 0; i < 4; ++i)
			points[i] = rotation * points[i] + translation;
	}
	Marker() {}
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
			optimize();
		}
        cameraPose = frame->pose;
#ifdef WITH_GUI
		cv::Mat leftRGB, rightRGB;
		cvtColor(left, leftRGB, cv::COLOR_BGRA2BGR);
		cvtColor(right, rightRGB, cv::COLOR_BGRA2BGR);
		drawMarkers<false>(leftRGB);
		drawMarkers<true>(rightRGB);
		cv::imshow("Left", leftRGB);
		cv::imshow("Right", rightRGB);
		cv::waitKey(1);
#endif
        return true;
    }
	template<bool right>
	void drawMarkers(cv::Mat &out)
	{
		const double
			fx = parent->cameraMatrix.at<double>(0, 0),
			fy = parent->cameraMatrix.at<double>(1, 1),
			cx = parent->cameraMatrix.at<double>(0, 2),
			cy = parent->cameraMatrix.at<double>(1, 2);

		auto icp = cameraPose.inverse();
		vector<vector<cv::Point2f>> corners;
		vector<int> ids;		
		for (const auto &m : markers) {
			ids.push_back(m.first);
			corners.emplace_back();
			auto &points = corners.back();
			for (size_t i = 0; i < 4; ++i) {
				auto pe = projectPoint<right>(icp * m.second.points[i]);
				cv::Point2f p2(float(pe.x() * fx + cx), float(pe.y() * fy + cy));
				points.push_back(p2);
			}
		}
		cv::aruco::drawDetectedMarkers(out, corners, ids);
	}
private:
	template<bool right>
	Vector2d inline projectPoint(const Vector3d &point)
	{
		return Vector2d(point.x() + parent->baseline * (right ? -0.5 : 0.5), point.y()) / point.z();
	}
    void imageToMarkers(const cv::Mat &image, std::vector<int> &markerIds, std::vector<std::vector<cv::Point2f>> &markerCorners/*, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs*/)
    {
        using namespace cv;
        cv::cvtColor(image, gray, COLOR_BGRA2GRAY);
        GaussianBlur(gray, gray, Size(7, 7), 0);
        aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters);
        //aruco::estimatePoseSingleMarkers(markerCorners, float(markerDefSize), parent->cameraMatrix, parent->distCoeffs, rvecs, tvecs);

        /*Mat rgb;
        cvtColor(image, rgb, COLOR_BGRA2BGR);
        aruco::drawDetectedMarkers(rgb, markerCorners, markerIds);
        for(size_t i=0; i < markerIds.size(); i++)
                cv::aruco::drawAxis(rgb, parent->cameraMatrix, parent->distCoeffs, rvecs[i], tvecs[i], 0.05);
        imshow("Markers", rgb);
        waitKey();*/
    }
	void createFrame(const cv::Mat &left, const cv::Mat &right, Frame &out)
	{
		using namespace std;
		vector<int> leftIds, rightIds;
		vector<vector<cv::Point2f>> leftCorners, rightCorners;
		//vector<cv::Vec3d> leftR, leftT, rightR, rightT;

		imageToMarkers(left, leftIds, leftCorners);// , leftR, leftT);
		imageToMarkers(right, rightIds, rightCorners);// , rightR, rightT);
		map<int, size_t> ids;
		for (size_t il = 0; il < leftIds.size(); ++il)
			ids.insert(make_pair(leftIds[il], il));
		out.markers.clear();
		for (size_t ir = 0; ir < rightIds.size(); ++ir) {
			auto it = ids.find(rightIds[ir]);
			if (it != ids.end()) {
				size_t il = it->second;
				out.markers.insert(make_pair(rightIds[ir], FrameMarker(leftCorners[il], rightCorners[ir], parent->cameraMatrix)));
			}
		}
	}
	void initializeByFrame(Frame &frame)
    {
        assert(!frame.markers.empty());
        const auto &marker = frame.markers.begin()->second;
		Vector3d points[4];
		Marker::convertPoints(marker.leftPoints, marker.rightPoints, points, parent->baseline);
		Vector3d
			x = points[1] - points[0] + points[3] - points[2],
			y = points[2] - points[0] + points[3] - points[1],
			z = x.cross(y);
		Matrix3d rm; rm << x, y, z;
		Quaterniond q(rm);
		SE3d pose(q, (points[0] + points[1] + points[2] + points[3]) * 0.25);
		frame.pose = pose.inverse();
		rm = frame.pose.rotationMatrix();
		Vector3d translation = frame.pose.translation();
        for (auto p : frame.markers) {
            auto ir = markers.insert(make_pair(p.first, Marker(p.second, rm, translation, parent->baseline)));
			//if (!ir.second)
			//	ir.first->second = p.second;
        }
    }
	template<bool right, typename Derived>
	inline void dUV_dPose(
		const Vector3d& framePoint,
		Block<Derived, 2, 6> &J)
	{
		const double x = framePoint[0] + parent->baseline * (right ? -0.5 : 0.5);
		const double y = framePoint[1];
		const double z = framePoint[2];
		const double z_inv = 1 / z;
		const double z_inv_2 = z_inv * z_inv;

		J(0, 0) = z_inv;              // -1/z
		J(0, 1) = 0.0;                 // 0
		J(0, 2) = -x * z_inv_2;           // x/z^2
		J(0, 3) = -x * y * z_inv_2;            // x*y/z^2
		J(0, 4) = 1.0 - x * J(0, 2);   // -(1.0 + x^2/z^2)
		J(0, 5) = -y * z_inv;             // y/z
		
		J(1, 0) = 0.0;                 // 0
		J(1, 1) = z_inv;              // -1/z
		J(1, 2) = -y * z_inv_2;           // y/z^2
		J(1, 3) = -1.0 + y * J(1, 2);      // 1.0 + y^2/z^2
		J(1, 4) = J(0, 3);             // -x*y/z^2
		J(1, 5) = -x * z_inv;            // x/z
	}
	template<bool right, typename Derived>
	inline void dUV_dPoint(
		const Vector3d& framePoint,
		const Matrix3d& rotation,
		Block<Derived, 2, 3> &J)
	{
		const double x = framePoint[0] + parent->baseline * (right ? -0.5 : 0.5);
		const double y = framePoint[1];
		const double z = framePoint[2];

		J = (Vector2d(x, y) * rotation.block<1, 3>(2, 0) - rotation.block<2, 3>(0, 0) * z) / (z * z);
	}
	void correct(VectorXd &dT, const map<int, int> &markerVars)
	{
		int frameCol = 0;
		for (auto &f : frames) {
			f.prevPose = f.pose;
			f.pose *= SE3d::exp(dT.segment<6>(frameCol));
			frameCol += 6;
			for (auto &m : f.markers) {
				auto &gm = markers.at(m.first);
				auto *worldPoints = gm.points;
				int col = markerVars.find(m.first)->second;
				for (size_t i = 0; i < 4; ++i) {
					gm.prevPoints[i] = worldPoints[i];
					worldPoints[i] += dT.segment<3>(col);
					col += 3;
				}
			}
		}
	}
	void correctPoses(VectorXd &dT)
	{
		int frameCol = 0;
		for (auto &f : frames) {
			f.prevPose = f.pose;
			f.pose *= SE3d::exp(dT.segment<6>(frameCol));
			frameCol += 6;
		}
	}
	void restore()
	{
		for (auto &f : frames)
			f.pose = f.prevPose;
		for (auto &m : markers)
			for (int i = 0; i < 4; ++i)
				m.second.points[i] = m.second.prevPoints[i];
	}
	void testJ(MatrixXd &J, const VectorXd &e, const map<int, int> &markerVars)
	{
		const double eps = 1E-6;
		VectorXd e1;
		MatrixXd J1;
		e1.resize(e.size());
		J1.resizeLike(J);
		VectorXd Jest;
		//Jest.resizeLike(e);
		int frameCol = 0;
		Vector6d dT = Vector6d::Zero();
		for (auto &f : frames) {
			auto t = f.pose;
			auto toFrame0 = f.pose.inverse();
			auto r0 = toFrame0.rotationMatrix();
			auto t0 = toFrame0.translation();
			//auto framePoint = r0 * wp + t0;
			for (size_t i = 0; i < 6; i++) {
				dT[i] = eps;
				//auto tr = f.pose.translation();

				f.pose *= SE3d::exp(dT);

				dT[i] = 0;
				calcJEpoints<false>(J1, e1, markerVars);
				Jest = (e1 - e) / eps;
				for (int row = 0; row < Jest.rows(); ++row) {
					double j = J(row, frameCol + i);
					double jest = Jest(row);
					if (abs(j - jest) > 1E-4) {
						//cout << "wrong j";
						J(row, frameCol + i) = jest;
					}
				}
				f.pose = t;
			}

			frameCol += 6;
			//f.pose = SE3d::exp(dT.segment<6>(col)) * f.pose;
			for (auto &m : f.markers) {
				auto *worldPoints = markers.at(m.first).points;
				int col = markerVars.find(m.first)->second;

				for (size_t i = 0; i < 4; ++i) {
					for (size_t d = 0; d < 3; ++d) {
						double t = worldPoints[i][d];
						worldPoints[i][d] += eps;
						calcJEpoints<false>(J1, e1, markerVars);
						worldPoints[i][d] = t;
						Jest = (e1 - e) / eps;
						for (int row = 0; row < Jest.rows(); ++row) {
							double j = J(row, col + d);
							double jest = Jest(row);
							if (abs(j - jest) > 1E-5) {
								//cout << "wrong j";
							}
						}
					}
					col += 3;
				}
			}
		}
	}

	template<bool withPose>
	void calcJEpoints(MatrixXd &J, VectorXd &e, const map<int, int> &markerVars)
	{
		assert(J.rows() == e.rows());
		int row = 0, frameCol = 0;
		for (auto &f : frames) {
			auto toFrame = f.pose.inverse();
			auto rotation = toFrame.rotationMatrix();
			auto translation = toFrame.translation();
			auto frameToWorld = f.pose.rotationMatrix();
			for (auto &m : f.markers) {
				const auto *worldPoints = markers.at(m.first).points;
				int col = markerVars.find(m.first)->second;
				for (size_t i = 0; i < 4; ++i) {
					auto framePoint = rotation * worldPoints[i] + translation;
					e.segment<2>(row) = m.second.leftPoints[i] - projectPoint<false>(framePoint);
					if (withPose)
						dUV_dPose<false>(framePoint, J.template block<2, 6>(row, frameCol));
					dUV_dPoint<false>(framePoint, rotation, J.template block<2, 3>(row, col));
					row += 2;
					e.segment<2>(row) = m.second.rightPoints[i] - projectPoint<true>(framePoint);
					if (withPose)
						dUV_dPose<true>(framePoint, J.template block<2, 6>(row, frameCol));
					dUV_dPoint<true>(framePoint, rotation, J.template block<2, 3>(row, col));
					row += 2;
					col += 3;
				}
			}
			if (withPose)
				frameCol += 6;
		}
		assert(row == J.rows());
	}
	void calcJEposes(MatrixXd &J, VectorXd &e)
	{
		assert(J.rows() == e.rows());
		int row = 0, frameCol = 0;
		for (auto &f : frames) {
			auto toFrame = f.pose.inverse();
			auto rotation = toFrame.rotationMatrix();
			auto translation = toFrame.translation();
			auto frameToWorld = f.pose.rotationMatrix();
			for (auto &m : f.markers) {
				const auto *worldPoints = markers.at(m.first).points;
				for (size_t i = 0; i < 4; ++i) {
					auto framePoint = rotation * worldPoints[i] + translation;
					e.segment<2>(row) = m.second.leftPoints[i] - projectPoint<false>(framePoint);
					dUV_dPose<false>(framePoint, J.template block<2, 6>(row, frameCol));
					row += 2;
					e.segment<2>(row) = m.second.rightPoints[i] - projectPoint<true>(framePoint);
					dUV_dPose<true>(framePoint, J.template block<2, 6>(row, frameCol));
					row += 2;
				}
			}
				frameCol += 6;
		}
		assert(row == J.rows());
	}
	bool optimize()
    {
		size_t 
			varCount = 0, 
			eqCount = 0;

		map<int, int> markerVars;
		for (const auto &f : frames) {
			for (const auto &m : f.markers) {
				auto inserted = markerVars.insert(make_pair(m.first, int(varCount)));
				if (inserted.second) {
					varCount += 12; // 4 points with 3 DoF
				}
				eqCount += 16; // 4 points with 2 coords on 2 images
			}
		}
		if (!eqCount)
			return false;
		
		MatrixXd Jpoints, Jpose;
		VectorXd e;
		Jpoints.resize(eqCount, varCount);
		Jpoints.setZero();
		Jpose.resize(eqCount, 6 * frames.size());
		Jpose.setZero();
		e.resize(eqCount);

		//std::cout << "calcErrors:" << std::endl;
		double ebest = 0;
		for (int it = 1; it < 10; it++) {
			calcJEposes(Jpose, e);
			//testJ(J, e, markerVars);
			double en = e.norm();
			cout << "Iteration " << it << " error: " << en << endl;
			if (ebest && (ebest <= en)) {
				restore();
				break;
			}
			else ebest = en;

			auto a = Jpose.transpose() * Jpose;
			auto b = Jpose.transpose() * -e;
			VectorXd dT = a.ldlt().solve(b);
			correctPoses(dT);
		}
 		return true;
    }
	bool optimizeFrame()
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

