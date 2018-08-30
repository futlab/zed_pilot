#include <string>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char **argv)
{
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    for (int x = 1; x < argc; ++x) {
        int id = atoi(argv[x]);
        cv::aruco::drawMarker(dictionary, id, 600, markerImage, 1);
#ifdef WITH_GUI
        cv::imshow("Marker", markerImage);
        if (cv::waitKey() == 'q')
            return 1;
#endif
        cv::imwrite(std::string(argv[x]) + ".png", markerImage);
    }
    return 0;
}
