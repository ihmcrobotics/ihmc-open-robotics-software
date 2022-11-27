#pragma once

#include "opencv4/opencv2/opencv.hpp"

#include "application_state.h"
#include "point_landmark.h"
#include "camera_model.h"
#include "keyframe.h"


using KeyPointVec = std::vector<cv::KeyPoint>;
using Point2fVec = std::vector<cv::Point2f>;
using PointLandmarkVec = std::vector<PointLandmark>;
using KeyframeVec = std::vector<Keyframe>;
using DMatchVec = std::vector<cv::DMatch>;

class VisualOdometry
{
   public:
      VisualOdometry(ApplicationState& app);
      
      void PreInitialize(KeyPointVec& kpCurLeft, KeyPointVec& kpCurRight, cv::Mat descCurLeft,  DMatchVec& stereoMatches);
      // cv::Mat Initialize(KeyPointVec& kpCurLeft, KeyPointVec& kpCurRight, DMatchVec& stereoMatches, KeyframeVec& keyframes, PointLandmarkVec& landmarks);
      bool UpdateStereo(cv::Mat& leftImage, cv::Mat& rightImage);
      // void UpdateMonocular(const cv::Mat& image);

      void InsertLandmarks(const PointLandmarkVec& landmarks);

      void InsertKeyframe(Eigen::Matrix4d pose, cv::Mat& descLeft, KeyPointVec& kpLeft, const std::vector<int>& kpIDs);
      void InsertKeyframe(Eigen::Matrix4d pose, const cv::Mat& descLeft, const cv::Mat& descRight, KeyPointVec& kpLeft, KeyPointVec& kpRight);
      void InsertKeyframe(Eigen::Matrix4d pose, cv::Mat& descLeft, cv::Mat& descRight, KeyPointVec& kpLeft, KeyPointVec& kpRight, const cv::Mat& leftMat, const cv::Mat& rightMat);

      bool UpdateStereoOld(const cv::Mat& leftImage, const cv::Mat& rightImage);

      void ExtractPoseLinear();
      void ExtractKeypoints_FAST(cv::Mat img_1, Point2fVec& points1);
      void ExtractKeypoints(cv::Mat img, KeyPointVec& points, cv::Mat& desc);
      void TrackKeypoints(cv::Mat prev, cv::Mat cur, Point2fVec& prev_pts, Point2fVec& cur_pts);
      void MatchKeypoints(cv::Mat& desc1, cv::Mat& desc2, DMatchVec& matches);
      void TransferKeypointIDs(const std::vector<int>& trainIDs, std::vector<int>& queryIDs, DMatchVec& matches);
      void GridSampleKeypoints(KeyPointVec& keypoints, DMatchVec& matches);
      void ExtractMatchesAsPoints(const KeyPointVec& keypoints, Point2fVec& points);
      void FilterMatchesByDistance(DMatchVec& matches, const KeyPointVec& kpTrain, const KeyPointVec& kpQuery, float distanceThreshold);
      void CalculateOdometry_ORB(Keyframe& kf, cv::Mat leftImage, cv::Mat rightImage, cv::Mat& cvPose, PointLandmarkVec& points3D);
      void CalculateOdometry_FAST(Eigen::Matrix4f& transform);
      
      void TriangulateLandmarksStereoNormal(const KeyPointVec& pointsTrain, const KeyPointVec& pointsQuery, 
                                          const cv::Mat& descTrain, const DMatchVec& matches,
                                          const std::vector<int>& kpIDs, PointLandmarkVec& points3D);
      void TriangulateKeypointsByDisparity(const KeyPointVec& kp, const cv::Mat& disparity, std::vector<Eigen::Vector3f>& points3d);
      void ExtractMatchesAsPoints(const KeyPointVec& kpTrain, const KeyPointVec& kpQuery, const DMatchVec& matches, Point2fVec& pointsTrain, Point2fVec& pointsQuery);

      cv::Mat EstimateMotionPnP(Point2fVec& points2d, const PointLandmarkVec& points3d, cv::Mat& mask, const CameraModel& cam);
      cv::Mat EstimateMotion(Point2fVec& prevFeatures, Point2fVec& curFeatures, cv::Mat& mask, const CameraModel& cam);
      cv::Mat TrackCameraPose(const KeyPointVec& kp, const cv::Mat& desc, const KeyframeVec& keyframes, const PointLandmarkVec& landmarks);
      cv::Mat TriangulatePoints(Point2fVec& prevPoints, Point2fVec& curPoints, const CameraModel& cam, cv::Mat relativePose);
      cv::Mat CalculateStereoDepth(cv::Mat left, cv::Mat right);
      

      // void DrawLandmarks(cv::Mat& img, PointLandmarkVec& landmarks);
      void DrawAllMatches(cv::Mat& image);
      void Display(cv::Mat& image);
      void Show(int delay = 1);
      void PrintKeyframeIDs();

      const Keyframe& GetLastKeyframe() const {return _keyframes[_keyframes.size() - 1]; };
      const PointLandmarkVec& GetLandmarkVec() const {return _curPoints3D; };

   private:
      ApplicationState _appState;
      Eigen::Matrix4f _cameraPose;

      bool _preInitialized = false;
      bool _initialized = false;
      
      float _scalar = 0.03f;
      uint32_t _count = 0;
      uint32_t _kFeatures = 400;
      uint32_t _kMinFeatures = 300;
      uint32_t _width = 1241;
      uint32_t _height = 376;
      uint32_t _xGridCount = 80;
      uint32_t _yGridCount = 30;

      bool _useExtrinsicGuess = false;
      int _iterationsCount = 100;
      float _reprojectionError = 2.0;
      double _confidence = 0.999;


      cv::Ptr<cv::StereoBM> _stereo = cv::StereoBM::create();
      cv::Ptr<cv::ORB> _orb;

      KeyframeVec _keyframes;
      PointLandmarkVec _landmarks;

      DMatchVec _matchesLeft, _matchesRight, _prevMatchesStereo, _curMatchesStereo;
      KeyPointVec _kpPrevLeft, _kpPrevRight, _kpCurLeft, _kpCurRight;
      PointLandmarkVec _prevPoints3D, _curPoints3D;
      Point2fVec _prevFeaturesLeft, _curFeaturesLeft;
      Point2fVec _prevPoints2D, _curPoints2D;
      
      cv::Mat _lastKeyframeImage;
      cv::Mat _curDisparity;
      cv::Mat _curFinalDisplay, _prevFinalDisplay;
      cv::Mat _curPoseLeft, curPoseRight;
      cv::Mat _descCurRight, _descCurLeft;
      
      cv::Mat _cvCurPose = cv::Mat::eye(4,4, CV_32F);

      CameraModel _leftCamera;
      CameraModel _rightCamera;

      double _baselineDistance = 0.5;

      int _uniqueKeypointID = -1;
      
};

