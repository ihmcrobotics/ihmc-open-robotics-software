#include "opencv_tools.h"

namespace OpenCVTools
{
    void DrawMatchesSingle(std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts, cv::Mat& outImage)
    {
        for (uint32_t i = 0; i < prev_pts.size(); i++)
        {
            cv::line(outImage, prev_pts[i], cur_pts[i], cv::Scalar(0, 255, 0), 3);
            cv::circle(outImage, prev_pts[i], 2, cv::Scalar(0, 0, 0), -1);
            cv::circle(outImage, cur_pts[i], 2, cv::Scalar(255, 255, 255), -1);
        }
    }


    void DrawMatchesDouble(cv::Mat& img1, const std::vector<cv::KeyPoint>& kp1, cv::Mat& img2, const std::vector<cv::KeyPoint>& kp2, std::vector<cv::DMatch>& matches, cv::Mat& outImage)
    {
        std::vector<cv::Mat> imgs = {img1, img2};
        cv::vconcat(imgs, outImage);
        cv::cvtColor(outImage, outImage, cv::COLOR_GRAY2BGR);
        
        uint8_t color = 0;
        for (cv::DMatch m : matches)
        {
            color++;
            cv::Point2f p1 = kp1[m.trainIdx].pt;
            cv::Point2f p2 = kp2[m.queryIdx].pt;

            cv::Scalar matchColor(color * 321 % 255, color * 136 % 255, color * 23 % 255);

            cv::line(outImage, {(int)p1.x, (int)p1.y}, {(int)p2.x, (int)p2.y + img1.rows}, matchColor, 2);
            cv::circle(outImage, {(int)p1.x, (int)p1.y}, 4, matchColor, 2);
            cv::circle(outImage, {(int)p2.x, (int)p2.y + img1.rows}, 4, matchColor, 2);
        }
    }

    void DisplayImage(const std::string& name, cv::Mat& image, int delay, float scale)
    {
        cv::resize(image, image, cv::Size((int)(scale * image.cols), (int)(scale * image.rows)));
        cv::imshow(name, image);
        cv::waitKey(delay);
    }
}