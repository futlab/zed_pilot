#include <string>
#include <opencv2/aruco.hpp>
#ifdef SHOW_RESULT
#include <opencv2/highgui.hpp>
#endif

int main(int argc, char **argv)
{
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    for (int x = 1; x < argc; ++x) {
        int id = atoi(argv[x]);
        cv::aruco::drawMarker(dictionary, id, 300, markerImage, 1);
#ifdef SHOW_RESULT
        cv::imshow("Marker", markerImage);
        if (cv::waitKey() == 'q')
            return 1;
#endif
        cv::imwrite(std::string(argv[x]) + ".png", markerImage);
    }
    return 0;
}
