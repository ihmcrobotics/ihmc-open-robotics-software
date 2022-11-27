#include "visual_odometry.h"
#include "opencv_tools.h"

#include <Eigen/Geometry>

VisualOdometry::VisualOdometry(ApplicationState& app) : _appState(app)
{
   _leftCamera.SetParams(718.856, 718.856, 607.193, 185.216);

   _cameraPose = Eigen::Matrix4f::Identity();

   _kFeatures = app.NUM_VISUAL_FEATURES;
   _kMinFeatures = app.MIN_NUM_VISUAL_FEATURES;

   _orb = cv::ORB::create(_kFeatures);

   _stereo->setNumDisparities(app.STEREO_NUM_DISPARITIES * 16);
   _stereo->setBlockSize(2 * app.STEREO_BLOCK_SIZE + 1);
   _stereo->setPreFilterSize(2 * app.STEREO_PRE_FILTER_SIZE + 1);
   _stereo->setPreFilterType(app.STEREO_PRE_FILTER_TYPE);
   _stereo->setPreFilterCap(app.STEREO_PRE_FILTER_CAP);
   _stereo->setMinDisparity(app.STEREO_MIN_DISPARITY);
   _stereo->setTextureThreshold(app.STEREO_TEXTURE_THRESHOLD);
   _stereo->setUniquenessRatio(app.STEREO_UNIQUENESS_RATIO);
   _stereo->setSpeckleRange(app.STEREO_SPECKLE_RANGE);
   _stereo->setSpeckleWindowSize(app.STEREO_SPECKLE_WINDOW_SIZE);
   _stereo->setDisp12MaxDiff(app.STEREO_DISP_12_MAX_DIFF);


}

bool VisualOdometry::UpdateStereo(cv::Mat& leftImage, cv::Mat& rightImage)
{
   printf("UpdateStereo\n");

   ExtractKeypoints(leftImage, _kpCurLeft, _descCurLeft);
   ExtractKeypoints(rightImage, _kpCurRight, _descCurRight);
   MatchKeypoints(_descCurLeft, _descCurRight, _curMatchesStereo);

   if(!_preInitialized)
   {
      PreInitialize(_kpCurLeft, _kpCurRight, _descCurLeft, _curMatchesStereo);
      return false;
   }

   // if(!_initialized)
   // {
   //    cv::Mat pose = Initialize(_kpCurLeft, _kpCurRight, _curMatchesStereo, _keyframes, _landmarks);
   //    Eigen::Map<Eigen::Matrix<float, 4, 4>, Eigen::RowMajor> eigPose(reinterpret_cast<float *>(pose.data));
   //    Eigen::Matrix4d initialRelativePose = eigPose.cast<double>();
   //    initialRelativePose.transposeInPlace();

   //    if (cameraPose.block<3, 1>(0, 3).norm() > 1)
   //    {

   //    }

   //    return false;
   // }
   else
   {
      Eigen::Matrix4f trackedPose = TrackCameraPose(_kpCurLeft, _descCurLeft, _keyframes, _landmarks);

      if (trackedPose.block<3, 1>(0, 3).norm() > 1)
      {
         TriangulateLandmarks(_keyframes, _landmarks);
      }

      auto lastKeyframe = _keyframes[_keyframes.size() - 1];
      cv::Mat descPrev = lastKeyframe.descLeft;
      KeyPointVec& kpPrev = lastKeyframe.keypointsLeft;

      DMatchVec matches;
      MatchKeypoints(descPrev, _descCurLeft, matches);
      printf("Total Matches to Keyframe: %ld\n", matches.size());

      GridSampleKeypoints(kpPrev, matches);
      printf("Total Matches After Grid Sampling: %ld\n", matches.size());

      FilterMatchesByDistance(matches, kpPrev, _kpCurLeft, 150.0f);
      printf("Total Matches After Distance Filter: %ld\n", matches.size());

      std::vector<int> keypointIDs(_kpCurLeft.size(), -1);
      TransferKeypointIDs(lastKeyframe.keypointIDs, keypointIDs, matches);
      printf("Total Keypoints: %ld\n", keypointIDs.size());

      Point2fVec pointsTrain;
      Point2fVec pointsQuery;
      ExtractMatchesAsPoints(kpPrev, _kpCurLeft, matches, pointsTrain, pointsQuery);
      printf("Total Motion Correspondences: %ld\n", pointsTrain.size());

      cv::Mat mask;
      cv::Mat pose = EstimateMotion(pointsTrain, pointsQuery, mask, _leftCamera);
      printf("Total Motion Correspondences (After Mask): %ld\n", pointsTrain.size());

      cv::Mat cvPose = pose;
      Eigen::Map<Eigen::Matrix<float, 4, 4>, Eigen::RowMajor> eigenPoseFloat(reinterpret_cast<float *>(cvPose.data));
      Eigen::Matrix4d eigenPose = eigenPoseFloat.cast<double>();
      eigenPose.transposeInPlace();

      Eigen::AngleAxisd rotation(eigenPose.block<3,3>(0,0));
      // eigenPose.block<3,3>(0,0) = rotation.toRotationMatrix();

      Eigen::Quaterniond quat(rotation);
      quat.normalize();
      eigenPose.block<3,3>(0,0) = quat.toRotationMatrix();

      _curPoints3D.clear();
      TriangulateLandmarksStereoNormal(_kpCurLeft, _kpCurRight, _descCurLeft, _curMatchesStereo, keypointIDs, _curPoints3D);
      printf("Total Landmarks Triangulated: %ld\n", _curPoints3D.size());

      std::cout << "Pose:" << std::endl << eigenPose << std::endl;
      std::cout << "Determinant: " << eigenPose.determinant() << std::endl;

      _lastKeyframeImage = leftImage.clone();
      InsertKeyframe(eigenPose, _descCurLeft, _kpCurLeft, keypointIDs);
      
      InsertLandmarks(_curPoints3D);
      
      PrintKeyframeIDs();

      // Visualization and Logging Only --------------------------------------------

      std::cout << "Camera Pose: " << std::endl << pose << std::endl;

      cv::Mat outImage;
      OpenCVTools::DrawMatchesDouble(_lastKeyframeImage, kpPrev, leftImage, _kpCurLeft, matches, outImage);
      OpenCVTools::DisplayImage("TestMatchKeypointsMonocular", outImage, 1);

      // Visualization and Logging Only --------------------------------------------
   }
   printf("Total Keyframes: %ld\n", _keyframes.size());

   return true;
}

void VisualOdometry::PrintKeyframeIDs()
{
   printf("Keyframes: [");
   for(auto keyframe : _keyframes)
   {
      printf("%d, ", keyframe.id);
   }
   printf("]\n");
}

void VisualOdometry::ExtractKeypoints_FAST(cv::Mat img_1, Point2fVec& points1)
{
   KeyPointVec keypoints_1;
   int fast_threshold = 20;
   bool nonmaxSuppression = true;
   cv::FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
   cv::KeyPoint::convert(keypoints_1, points1, std::vector<int>());
}

void VisualOdometry::ExtractKeypoints(cv::Mat img, KeyPointVec& points, cv::Mat& desc)
{
   desc.setTo(0);
   points.clear();
   _orb->detectAndCompute(img, cv::noArray(), points, desc);
}


void VisualOdometry::MatchKeypoints(cv::Mat& descTrain, cv::Mat& descQuery, DMatchVec& matches)
{
   matches.clear();
   using namespace cv;
   //   BFMatcher matcher(NORM_HAMMING, true);
   //   matcher.match( descQuery, descTrain, matches);

   //   std::sort(matches.begin(), matches.end(), [&](const cv::DMatch& a, const cv::DMatch& b)
   //   {
   //      return a.distance < b.distance;
   //   });
   //
   //   if(matches.size() > 1200) matches.resize(1200);

   static cv::Ptr<cv::DescriptorMatcher> bf_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
   //   Ptr<DescriptorMatcher> flannMatcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
   std::vector<std::vector<DMatch> > knn_matches;
   bf_matcher->knnMatch(descQuery, descTrain, knn_matches, 2);
   //-- Filter matches using the Lowe's ratio test
   const float ratio_thresh = 0.8f;
   for (size_t i = 0; i < knn_matches.size(); i++)
   {
      if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
      {
         matches.push_back(knn_matches[i][0]);
      }
   }

   //   for(auto match : matches)
   //      printf("Match Distance: {}", match.distance);

   //-- Filter matches
   //   DMatchVec finalMatches;
   //   for (size_t i = 0; i < matches.size(); i++)
   //   {
   //         if(matches[i].distance < 40)
   //            finalMatches.push_back(matches[i]);
   //   }
   //   matches = finalMatches

   //   printf("MatchKeypoints(): Total Matches: {}", matches.size());
}

void VisualOdometry::TransferKeypointIDs(const std::vector<int>& trainIDs, std::vector<int>& queryIDs, DMatchVec& matches)
{
   for(uint32_t i = 0; i<matches.size(); i++)
   {
      if(trainIDs[matches[i].trainIdx] != -1)
      {
         queryIDs[matches[i].queryIdx] = trainIDs[matches[i].trainIdx];
      }
   }
}

void VisualOdometry::GridSampleKeypoints(KeyPointVec& keypoints, DMatchVec& matches)
{
   uint32_t xStep = _width / _xGridCount;
   uint32_t yStep = _height / _yGridCount;

   bool grid[_yGridCount][_xGridCount];
   memset(grid, false, sizeof(bool) * _yGridCount * _xGridCount);

   DMatchVec finalMatches;
   for (uint32_t i = 0; i < matches.size(); i++)
   {
      cv::Point2f point(keypoints[matches[i].trainIdx].pt);

      if (point.x >= 0 && point.x < _width && point.y >= 0 && point.y < _height)
      {
         uint32_t xIndex = (uint32_t) ((float) point.x / (float) xStep);
         uint32_t yIndex = (uint32_t) ((float) point.y / (float) yStep);
         // printf("i: %d, Size: %ld Dims:%d %d Point: %.3lf %.3lf -> %d %d\n", i, matches.size(), width, height, point.x, point.y, xIndex, yIndex);

         if (xIndex < _xGridCount && yIndex < _yGridCount)
         {
            if (!grid[yIndex][xIndex])
            {
               finalMatches.push_back(matches[i]);
               grid[yIndex][xIndex] = true;
            }
         }
      }
   }

   matches.clear();
   matches = finalMatches;
}



void VisualOdometry::TriangulateLandmarksStereoNormal(const KeyPointVec& pointsTrain, const KeyPointVec& pointsQuery, 
                                                      const cv::Mat& descTrain, const DMatchVec& matches,
                                                      const std::vector<int>& kpIDs, PointLandmarkVec& points3D)
{
   points3D.clear();
   float x1, x2, y1, y2, y_hat, X, Y, Z;
   float cx = 607.1928, cy = 185.2157;
   for (auto match: matches)
   {
      //      printf("{} {}", pointsTrain[match.trainIdx].pt.x, pointsQuery[match.queryIdx].pt.x);

      /*
       * Left = 1, Right = 2
       * Z = f * B / (x1 - x2)
       * X = Z / (x1 * f) = x1 * B / (x1 - x2)
       * Y = X * (y_hat) / x1 = (y_hat / 2) * (B / (x1 - x2))
       * */

      x1 = pointsTrain[match.trainIdx].pt.x - cx;
      x2 = pointsQuery[match.queryIdx].pt.x - cx;
      y1 = pointsTrain[match.trainIdx].pt.y - cy;
      y2 = pointsQuery[match.queryIdx].pt.y - cy;

      if (abs(x1 - x2) > 1.0f && abs(y1 - y2) < 10.0f)
      {
         y_hat = (y1 + y2) / 2;

         Z = _leftCamera._fx * _baselineDistance / (x1 - x2);
         X = x1 * _baselineDistance / (x1 - x2);
         Y = y_hat * (_baselineDistance / (x1 - x2));

         //         printf("Point3D: {} {} {}", X, Y, Z);
         if (Z > 0)
         {
            Eigen::Vector3f point(X, Y, Z);
            Eigen::Vector2f measurement(pointsTrain[match.trainIdx].pt.x, pointsTrain[match.trainIdx].pt.y);

            // TODO: Add feature descriptor into the landmark.

            points3D.push_back({point, measurement, kpIDs[match.trainIdx]});
         }
      }
   }

   printf("TriangulateLandmarks: Descriptors -> (%d, %d)\n", descTrain.rows, descTrain.cols);

   //   printf("Triangulate(): Total Depth Points: {}", points3D.size());
}

cv::Mat
VisualOdometry::EstimateMotion(Point2fVec& prevFeatures, Point2fVec& curFeatures, cv::Mat& mask, const CameraModel& cam)
{
   using namespace cv;
   float data[9] = {cam._fx, 0, cam._cx, 0, cam._fy, cam._cy, 0, 0, 1};
   cv::Mat K = cv::Mat(3, 3, CV_32FC1, data);
   cv::Mat R(3, 3, CV_32FC1);
   cv::Mat t(1, 3, CV_32FC1);

   cv::Mat E = findEssentialMat(prevFeatures, curFeatures, K, cv::RANSAC, 0.999, 1.0, mask);


   std::cout << E << std::endl;

   recoverPose(E, prevFeatures, curFeatures, K, R, t, mask);

   for (int i = (uint32_t) prevFeatures.size() - 1; i >= 0; i--)
   {
      if ((int) mask.at<unsigned char>(i, 0) == 1)
      {
         prevFeatures.erase(prevFeatures.begin() + i);
         curFeatures.erase(curFeatures.begin() + i);
      }
   }

   Mat cvPose = Mat::eye(4, 4, CV_32FC1);
   R.copyTo(cvPose(Range(0, 3), Range(0, 3))); /* Excludes the 'end' element */
   t.copyTo(cvPose(Range(0, 3), Range(3, 4)));
   cv::invert(cvPose, cvPose);
   
   return cvPose;
}


cv::Mat VisualOdometry::EstimateMotionPnP(Point2fVec& points2d, const PointLandmarkVec& points3d, cv::Mat& mask, const CameraModel& cam)
{

   float data[9] = {cam._fx, 0, cam._cx, 0, cam._fy, cam._cy, 0, 0, 1};
   cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, data);

   std::vector<cv::Point3f> objectPoints;

   for (auto landmark: points3d)
   {
      objectPoints.emplace_back(cv::Point3f(landmark.GetPoint3D().x(), landmark.GetPoint3D().y(), landmark.GetPoint3D().z()));
   }

   std::cout << "There are " << points2d.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;

   cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
   distCoeffs.at<double>(0) = 0;
   distCoeffs.at<double>(1) = 0;
   distCoeffs.at<double>(2) = 0;
   distCoeffs.at<double>(3) = 0;

   cv::Mat rvec(3, 1, cv::DataType<double>::type);
   cv::Mat tvec(3, 1, cv::DataType<double>::type);
   cv::Mat rotation(3, 3, CV_32F);

   cv::solvePnPRansac(objectPoints, points2d, cameraMatrix, distCoeffs, rvec, tvec, 
                        _useExtrinsicGuess, _iterationsCount, _reprojectionError, _confidence, mask);
   cv::Rodrigues(rvec, rotation);

   cv::Mat cvPose = cv::Mat::eye(4, 4, CV_32FC1);
   rotation.copyTo(cvPose(cv::Range(0, 3), cv::Range(0, 3))); /* Excludes the 'end' element */
   tvec.copyTo(cvPose(cv::Range(0, 3), cv::Range(3, 4)));

   cv::invert(cvPose, cvPose);
   _cvCurPose = (_cvCurPose * cvPose);

   std::cout << "Pose: " << std::endl << cvPose << std::endl;

   // printf("%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\n", cvPose.at<float>(0), cvPose.at<float>(1), cvPose.at<float>(2),
   //       cvPose.at<float>(3), cvPose.at<float>(4), cvPose.at<float>(5), cvPose.at<float>(6), cvPose.at<float>(7), cvPose.at<float>(8), cvPose.at<float>(9),
   //       cvPose.at<float>(10), cvPose.at<float>(11));

   return cvPose;
}

void VisualOdometry::FilterMatchesByDistance(DMatchVec& matches, const KeyPointVec& kpTrain, const KeyPointVec& kpQuery, float distanceThreshold)
{
   for (int i = matches.size() - 1; i >= 0; i--)
   {
      auto m = matches[i];
      float dist = cv::norm(kpQuery[m.queryIdx].pt - kpTrain[m.trainIdx].pt);
      if (dist > distanceThreshold)
      {
         matches.erase(matches.begin() + i);
      }
   }
}

void VisualOdometry::Show(int delay)
{
   if (!_prevFinalDisplay.empty() && _prevFinalDisplay.rows != 0 && _prevFinalDisplay.cols != 0)
      cv::imshow("Previous Keypoints Visualizer", _prevFinalDisplay);
   if (!_curFinalDisplay.empty() && _curFinalDisplay.rows != 0 && _curFinalDisplay.cols != 0)
      cv::imshow("Keypoints Visualizer", _curFinalDisplay);
   cv::waitKey(delay);
}



// TODO: Move to OpenCV tools class
cv::Mat VisualOdometry::CalculateStereoDepth(cv::Mat left, cv::Mat right)
{
   cv::Mat disparity;
   _stereo->compute(left, right, disparity);
   disparity.convertTo(disparity, CV_8U, 1.0);
   return disparity;
}

void VisualOdometry::Display(cv::Mat& image)
{
   _curFinalDisplay = image.clone();
}

void VisualOdometry::ExtractMatchesAsPoints(const KeyPointVec& kpTrain, const KeyPointVec& kpQuery, const DMatchVec& matches, Point2fVec& pointsTrain, Point2fVec& pointsQuery)
{
   pointsTrain.clear();
   pointsQuery.clear();
   
   for (auto m: matches)
   {
      pointsTrain.emplace_back(cv::Point2f(kpTrain[m.trainIdx].pt));
      pointsQuery.emplace_back(cv::Point2f(kpQuery[m.queryIdx].pt));
   }
}


void VisualOdometry::TriangulateKeypointsByDisparity(const KeyPointVec& kp, const cv::Mat& depth, std::vector<Eigen::Vector3f>& points3d)
{
   std::cout << "Type: " << OpenCVTools::GetTypeString(depth.type()) << std::endl;

   for(uint32_t i = 0; i<kp.size(); i++)
   {
      Eigen::Vector3f point;
      points3d.emplace_back(std::move(point));
   }
}

// cv::Mat Initialize(KeyPointVec& kpCurLeft, KeyPointVec& kpCurRight, DMatchVec& stereoMatches, KeyframeVec& keyframes, PointLandmarkVec& landmarks)
// {

// }

void VisualOdometry::PreInitialize(KeyPointVec& kpCurLeft, KeyPointVec& kpCurRight, cv::Mat descCurLeft,  DMatchVec& stereoMatches)
{
   printf("Initialize()\n");

   printf("Assigning Unique Labels: %ld\n", kpCurLeft.size());
   std::vector<int> kpIDs(kpCurLeft.size(), -1);
   for(int i = 0; i<(int)kpCurLeft.size(); i++)
   {
      _uniqueKeypointID++;
      kpIDs[i] = _uniqueKeypointID;
   }

   printf("Triangulating Stereo Keypoints\n");
   _curPoints3D.clear();
   TriangulateLandmarksStereoNormal(kpCurLeft, kpCurRight, descCurLeft, stereoMatches, kpIDs, _curPoints3D);

   printf("Inserting Keyframe Initial\n");
   InsertKeyframe(Eigen::Matrix4d::Identity(), descCurLeft, kpCurLeft, kpIDs);
   InsertLandmarks(_curPoints3D);
   PrintKeyframeIDs();

   _preInitialized = true;
}


cv::Mat VisualOdometry::TrackCameraPose(const KeyPointVec& kp, const cv::Mat& desc, const Keyframe& lastKF, const PointLandmarkVec& landmarks)
{

   printf("TrackCameraPose: (KFs: %ld, KP:%ld, Desc:(%d, %d), Landmarks:%ld)\n", lastKF.id, 
            kp.size(), desc.rows, desc.cols, landmarks.size());

   _leftCamera.SetTransform(lastKF.pose.inverse());
   // Create a list of possible landmark IDs to project onto the camera view

   PointLandmarkVec candidateMapPoints;
   for(auto landmark : landmarks)
   {
      Eigen::Vector3f camPoint;
      Eigen::Vector2f imgPoint;
      bool hasValidZ = _leftCamera.Project(landmark.GetPoint3D(), camPoint, imgPoint);
      
      if(hasValidZ) 
      {
         candidateMapPoints.emplace_back(landmark);
      }
   }


   // Extract all landmark points that fall on the current camera view

   // Match 3D descriptors to 2D descriptors
   DMatchVec matchesToMap;
   MatchKeypointsToLandmarks(_kpCurLeft, _descCurLeft, candidateMapPoints, matchesToMap);


   // TODO: Extract 2D point vector for PnP estimation.
   

   // Estimate world-frame camera pose using PnP
   cv::Mat cameraWorldPose = EstimateMotionPnP();

   return cameraWorldPose;
   
}

// void VisualOdometry::UpdateLocalMap(const cv::Mat& relativePose, )
// {
//    if(!mapInitialized && _landmarks.size() == 0 && _keyframes.size() == 1)
//    {
      
//    }
//    else
//    {

//    }
// }

void VisualOdometry::InsertKeyframe(Eigen::Matrix4d pose, const cv::Mat& descLeft, const cv::Mat& descRight, KeyPointVec& kpLeft, KeyPointVec& kpRight)
{
   _keyframes.emplace_back(Keyframe(_keyframes.size(), pose, descLeft.clone(), descRight.clone(), kpLeft, kpRight));
}

void VisualOdometry::InsertKeyframe(Eigen::Matrix4d pose, cv::Mat& descLeft, cv::Mat& descRight, KeyPointVec& kpLeft, KeyPointVec& kpRight, const cv::Mat& leftMat, const cv::Mat& rightMat)
{
   _keyframes.emplace_back(Keyframe(_keyframes.size(), pose, descLeft.clone(), descRight.clone(), kpLeft, kpRight, leftMat.clone(), rightMat.clone()));
}

void VisualOdometry::InsertKeyframe(Eigen::Matrix4d pose, cv::Mat& descLeft, KeyPointVec& kpLeft, const std::vector<int>& kpIDs)
{
   _keyframes.emplace_back(Keyframe(_keyframes.size(), pose, descLeft.clone(), kpLeft, kpIDs));
}

void VisualOdometry::InsertLandmarks(const PointLandmarkVec& landmarks)
{
   for(auto landmark : landmarks)
   {
      _landmarks.emplace_back(landmark);
   }
}