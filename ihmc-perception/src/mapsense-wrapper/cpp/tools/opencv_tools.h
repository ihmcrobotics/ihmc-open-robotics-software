#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/highgui.hpp"

namespace OpenCVTools
{
    

    void DisplayImage(const std::string& name, cv::Mat& image, int delay = 1, float scale = 1.0f);
    void DrawMatchesSingle(std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts, cv::Mat& outImage);
    void DrawMatchesDouble(cv::Mat& img1, const std::vector<cv::KeyPoint>& kp1, cv::Mat& img2, const std::vector<cv::KeyPoint>& kp2, std::vector<cv::DMatch>& matches, cv::Mat& outImage);
}