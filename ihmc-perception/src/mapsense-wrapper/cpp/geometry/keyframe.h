#pragma once

struct Keyframe
{
   public:
      Keyframe(cv::Mat desc, std::vector<cv::KeyPoint> kp, Eigen::Matrix4f pose)
         : descLeft(desc), keypointsLeft(kp), pose(pose) {};
      Keyframe(cv::Mat descLeft, cv::Mat descRight, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, Eigen::Matrix4f pose, cv::Mat left, cv::Mat right)
            : descLeft(descLeft), descRight(descRight), keypointsLeft(kpLeft), keypointsRight(kpRight), pose(pose), leftImage(left), rightImage(right) {};
      cv::Mat descLeft, descRight;
      std::vector<cv::KeyPoint> keypointsLeft, keypointsRight;
      Eigen::Matrix4f pose;
      cv::Mat leftImage, rightImage;

};