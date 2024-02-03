#pragma once

#include "opencv4/opencv2/opencv.hpp"

using KeyPointVec = std::vector<cv::KeyPoint>;
using Point2fVec = std::vector<cv::Point2f>;
using DMatchVec = std::vector<cv::DMatch>;

class FeatureExtractor
{
    public:
        FeatureExtractor(uint32_t kFeatures);
        void ExtractKeypoints_FAST(cv::Mat img_1, Point2fVec& points1);
        void ExtractKeypoints(cv::Mat img, KeyPointVec& points, cv::Mat& desc);
        void MatchKeypoints(cv::Mat& desc1, cv::Mat& desc2, DMatchVec& matches);

    private:
        // cv::Ptr<cv::DescriptorMatcher> bf_matcher;
        cv::Ptr<cv::ORB> _orb;
};

