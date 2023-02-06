#pragma once

using KeyPointVec = std::vector<cv::KeyPoint>;
using Point2fVec = std::vector<cv::Point2f>;
using Point3fVec = std::vector<cv::Point3f>;

struct Keyframe
{
   public:

      Keyframe(uint32_t kid, Eigen::Matrix4d pose, cv::Mat desc, KeyPointVec kp, const std::vector<int>& ids)
         : id(kid), pose(pose), descLeft(desc), keypointsLeft(kp), keypointIDs(ids)  {};

      Keyframe(uint32_t kid, Eigen::Matrix4d pose, cv::Mat descLeft, cv::Mat descRight, KeyPointVec kpLeft, KeyPointVec kpRight, cv::Mat left, cv::Mat right)
            : id(kid), pose(pose), descLeft(descLeft), descRight(descRight), keypointsLeft(kpLeft), keypointsRight(kpRight), leftImage(left), rightImage(right) {};
      
      Keyframe(uint32_t kid, Eigen::Matrix4d pose, cv::Mat descLeft, cv::Mat descRight, KeyPointVec kpLeft, KeyPointVec kpRight)
            : id(kid), pose(pose), descLeft(descLeft), descRight(descRight), keypointsLeft(kpLeft), keypointsRight(kpRight) {};
      
      uint32_t id = 0;

      Eigen::Matrix4d pose;

      cv::Mat descLeft;
      cv::Mat descRight;
      
      KeyPointVec keypointsLeft;
      KeyPointVec keypointsRight;
      
      std::vector<int> keypointIDs;

      cv::Mat leftImage;
      cv::Mat rightImage;


};