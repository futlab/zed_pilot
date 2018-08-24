#ifndef ARUCO_H
#define ARUCO_H
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

class Aruco_;

class Aruco
{
private:
    Aruco_ *aruco_;
public:
    cv::Mat cameraMatrix, distCoeffs;
    double baseline;
    Aruco();
    ~Aruco();
    bool process(const cv::Mat &left, const cv::Mat &right);
};

#endif // ARUCO_H
