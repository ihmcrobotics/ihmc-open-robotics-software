#pragma once

using KeyPointVec = std::vector<cv::KeyPoint>;
using Point2fVec = std::vector<cv::Point2f>;

struct Keyframe
{
   public:

      Keyframe(uint32_t kid, Eigen::Matrix4f pose, cv::Mat desc, KeyPointVec kp, cv::Mat left)
         : id(kid), pose(pose), descLeft(desc), keypointsLeft(kp), leftImage(left)  {};

      Keyframe(uint32_t kid, Eigen::Matrix4f pose, cv::Mat descLeft, cv::Mat descRight, KeyPointVec kpLeft, KeyPointVec kpRight, cv::Mat left, cv::Mat right)
            : id(kid), pose(pose), descLeft(descLeft), descRight(descRight), keypointsLeft(kpLeft), keypointsRight(kpRight), leftImage(left), rightImage(right) {};
      
      Keyframe(uint32_t kid, Eigen::Matrix4f pose, cv::Mat descLeft, cv::Mat descRight, KeyPointVec kpLeft, KeyPointVec kpRight)
            : id(kid), pose(pose), descLeft(descLeft), descRight(descRight), keypointsLeft(kpLeft), keypointsRight(kpRight) {};
      
      Eigen::Matrix4f pose;

      cv::Mat descLeft;
      cv::Mat descRight;
      
      KeyPointVec keypointsLeft;
      KeyPointVec keypointsRight;
      
      cv::Mat leftImage;
      cv::Mat rightImage;

      uint32_t id = 0;

};