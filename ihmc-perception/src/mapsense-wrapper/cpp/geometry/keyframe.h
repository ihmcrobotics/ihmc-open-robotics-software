#pragma once

using KeyPointVec = std::vector<cv::KeyPoint>;
using Point2fVec = std::vector<cv::Point2f>;

struct Keyframe
{
   public:

      Keyframe(Eigen::Matrix4f pose, cv::Mat desc, KeyPointVec kp, cv::Mat left)
         : pose(pose), descLeft(desc), keypointsLeft(kp), leftImage(left)  {};

      Keyframe(Eigen::Matrix4f pose, cv::Mat descLeft, cv::Mat descRight, KeyPointVec kpLeft, KeyPointVec kpRight, cv::Mat left, cv::Mat right)
            : pose(pose), descLeft(descLeft), descRight(descRight), keypointsLeft(kpLeft), keypointsRight(kpRight), leftImage(left), rightImage(right) {};
      
      Keyframe(Eigen::Matrix4f pose, cv::Mat descLeft, cv::Mat descRight, KeyPointVec kpLeft, KeyPointVec kpRight)
            : pose(pose), descLeft(descLeft), descRight(descRight), keypointsLeft(kpLeft), keypointsRight(kpRight) {};
      
      Eigen::Matrix4f pose;

      cv::Mat descLeft;
      cv::Mat descRight;
      
      KeyPointVec keypointsLeft;
      KeyPointVec keypointsRight;
      
      cv::Mat leftImage;
      cv::Mat rightImage;

};