#pragma once

struct Keyframe
{
   public:

      Keyframe(Eigen::Matrix4f pose, cv::Mat desc, std::vector<cv::KeyPoint> kp)
         : pose(pose), descLeft(desc), keypointsLeft(kp)  {};

      Keyframe(Eigen::Matrix4f pose, cv::Mat descLeft, cv::Mat descRight, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, cv::Mat left, cv::Mat right)
            : pose(pose), descLeft(descLeft), descRight(descRight), keypointsLeft(kpLeft), keypointsRight(kpRight), leftImage(left), rightImage(right) {};
      
      Keyframe(Eigen::Matrix4f pose, cv::Mat descLeft, cv::Mat descRight, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight)
            : pose(pose), descLeft(descLeft), descRight(descRight), keypointsLeft(kpLeft), keypointsRight(kpRight) {};
      
      Eigen::Matrix4f pose;

      cv::Mat descLeft;
      cv::Mat descRight;
      
      std::vector<cv::KeyPoint> keypointsLeft;
      std::vector<cv::KeyPoint> keypointsRight;
      
      cv::Mat leftImage;
      cv::Mat rightImage;


};