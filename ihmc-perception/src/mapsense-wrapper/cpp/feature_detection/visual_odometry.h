#pragma once

#include "opencv4/opencv2/opencv.hpp"

#include "application_state.h"
#include "point_landmark.h"
#include "camera_model.h"
#include "keyframe.h"


using KeyPointVec = std::vector<cv::KeyPoint>;
using Point2fVec = std::vector<cv::Point2f>;
using PointLandmarkVec = std::vector<PointLandmark>;

class VisualOdometry
{
   public:
      VisualOdometry(ApplicationState& app);
      
      void Initialize(cv::Mat& leftImageCur, cv::Mat& rightImageCur);
      bool UpdateStereo(cv::Mat& leftImage, cv::Mat& rightImage);
      // void UpdateMonocular(const cv::Mat& image);
      void UpdateStereoExternal(cv::Mat& leftImageCur, cv::Mat& rightImageCur);

      void InsertKeyframe(Eigen::Matrix4f pose, cv::Mat& descLeft, KeyPointVec& kpLeft, const std::vector<int>& kpIDs, const cv::Mat& leftMat);

      void InsertKeyframe(Eigen::Matrix4f pose, const cv::Mat& descLeft, const cv::Mat& descRight, KeyPointVec& kpLeft, KeyPointVec& kpRight);
      void InsertKeyframe(Eigen::Matrix4f pose, cv::Mat& descLeft, cv::Mat& descRight, KeyPointVec& kpLeft, KeyPointVec& kpRight, const cv::Mat& leftMat, const cv::Mat& rightMat);
      void InsertKeyframe(Eigen::Matrix4f pose, cv::Mat& descLeft, KeyPointVec& kpLeft, const cv::Mat& leftMat);

      bool UpdateStereoOld(const cv::Mat& leftImage, const cv::Mat& rightImage);

      void ExtractPoseLinear();
      void ExtractKeypoints_FAST(cv::Mat img_1, Point2fVec& points1);
      void ExtractKeypoints(cv::Mat img, KeyPointVec& points, cv::Mat& desc);
      void TrackKeypoints(cv::Mat prev, cv::Mat cur, Point2fVec& prev_pts, Point2fVec& cur_pts);
      void MatchKeypoints(cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& matches);
      void TransferKeypointIDs(const std::vector<int>& trainIDs, std::vector<int>& queryIDs, std::vector<cv::DMatch>& matches);
      void GridSampleKeypoints(KeyPointVec& keypoints, std::vector<cv::DMatch>& matches);
      void ExtractMatchesAsPoints(const KeyPointVec& keypoints, Point2fVec& points);
      void FilterMatchesByDistance(std::vector<cv::DMatch>& matches, const KeyPointVec& kpTrain, const KeyPointVec& kpQuery, float distanceThreshold);
      // void ExtractFinalSet(std::vector<cv::DMatch> leftMatches, KeyPointVec curLeftKp, PointLandmarkVec& points3D);
      void CalculateOdometry_ORB(Keyframe& kf, cv::Mat leftImage, cv::Mat rightImage, cv::Mat& cvPose, PointLandmarkVec& points3D);
      void CalculateOdometry_FAST(Eigen::Matrix4f& transform);
      void TriangulateStereoNormal(KeyPointVec& pointsTrain, KeyPointVec& pointsQuery, std::vector<cv::DMatch>& matches,
                                   PointLandmarkVec& points3D);
      void TriangulateKeypointsByDisparity(const KeyPointVec& kp, const cv::Mat& disparity, std::vector<Eigen::Vector3f>& points3d);
      void ExtractMatchesAsPoints(const KeyPointVec& kpTrain, const KeyPointVec& kpQuery, const std::vector<cv::DMatch>& matches, Point2fVec& pointsTrain, Point2fVec& pointsQuery);


      cv::Mat EstimateMotion(Point2fVec& prevFeatures, Point2fVec& curFeatures, cv::Mat& mask, const CameraModel& cam);
      cv::Mat TriangulatePoints(Point2fVec& prevPoints, Point2fVec& curPoints, const CameraModel& cam, cv::Mat relativePose);
      cv::Mat CalculateStereoDepth(cv::Mat left, cv::Mat right);
      

      // void DrawLandmarks(cv::Mat& img, PointLandmarkVec& landmarks);
      void DrawAllMatches(cv::Mat& image);
      void Display(cv::Mat& image);
      void Show(int delay = 1);

      const Keyframe& GetLastKeyframe() const {return _keyframes[_keyframes.size() - 1]; };
      const PointLandmarkVec& GetLandmarkVec() const {return _curPoints3D; };

   private:
      ApplicationState _appState;
      Eigen::Matrix4f cameraPose;

      bool _initialized = false;
      float scalar = 0.03f;
      uint32_t count = 0;
      uint32_t kFeatures = 1200;
      uint32_t kMinFeatures = 1000;
      uint32_t width = 0;
      uint32_t height = 0;
      uint32_t xGridCount = 60;
      uint32_t yGridCount = 30;


      cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
      cv::Ptr<cv::ORB> _orb = cv::ORB::create(kFeatures);

      std::vector<Keyframe> _keyframes;
      std::vector<cv::DMatch> matchesLeft, matchesRight, prevMatchesStereo, curMatchesStereo;
      KeyPointVec kp_prevLeft, kp_prevRight, kp_curLeft, kp_curRight;
      PointLandmarkVec _prevPoints3D, _curPoints3D;
      Point2fVec prevFeaturesLeft, curFeaturesLeft;
      Point2fVec prevPoints2D, curPoints2D;
      
      cv::Mat curDisparity;
      cv::Mat curFinalDisplay, prevFinalDisplay;
      cv::Mat curPoseLeft, curPoseRight;
      cv::Mat desc_curRight, desc_curLeft;
      
      cv::Mat cvCurPose = cv::Mat::eye(4,4, CV_32F);

      CameraModel leftCamera;
      CameraModel rightCamera;

      double baselineDistance = 0.5;

      int uniqueKeypointID = -1;
      
};

