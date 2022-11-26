#include "visual_odometry.h"
#include "opencv_tools.h"

#include <Eigen/Geometry>

VisualOdometry::VisualOdometry(ApplicationState& app) : _appState(app)
{
   leftCamera.SetParams(718.856, 718.856, 607.193, 185.216);

   cameraPose = Eigen::Matrix4f::Identity();

   kFeatures = app.NUM_VISUAL_FEATURES;
   kMinFeatures = app.MIN_NUM_VISUAL_FEATURES;

   _orb = cv::ORB::create(kFeatures);

   stereo->setNumDisparities(app.STEREO_NUM_DISPARITIES * 16);
   stereo->setBlockSize(2 * app.STEREO_BLOCK_SIZE + 1);
   stereo->setPreFilterSize(2 * app.STEREO_PRE_FILTER_SIZE + 1);
   stereo->setPreFilterType(app.STEREO_PRE_FILTER_TYPE);
   stereo->setPreFilterCap(app.STEREO_PRE_FILTER_CAP);
   stereo->setMinDisparity(app.STEREO_MIN_DISPARITY);
   stereo->setTextureThreshold(app.STEREO_TEXTURE_THRESHOLD);
   stereo->setUniquenessRatio(app.STEREO_UNIQUENESS_RATIO);
   stereo->setSpeckleRange(app.STEREO_SPECKLE_RANGE);
   stereo->setSpeckleWindowSize(app.STEREO_SPECKLE_WINDOW_SIZE);
   stereo->setDisp12MaxDiff(app.STEREO_DISP_12_MAX_DIFF);


}

// void VisualOdometry::UpdateMonocular(const cv::Mat& image)
// {
//    width = (uint32_t)image.cols;
//    height = (uint32_t)image.rows;
//    cvtColor(image, prevLeft, cv::COLOR_BGR2GRAY);
//    ExtractKeypoints(prevLeft, kp_prevLeft, desc_prevLeft);
// }

bool VisualOdometry::UpdateStereo(cv::Mat& leftImage, cv::Mat& rightImage)
{
   printf("UpdateStereo\n");
   if(!_initialized)
   {
      Initialize(leftImage, rightImage);
      return false;
   }
   else
   {

      auto lastKeyframe = _keyframes[_keyframes.size() - 1];
      cv::Mat descPrev = lastKeyframe.descLeft;
      KeyPointVec& kpPrev = lastKeyframe.keypointsLeft;

      cv::Mat descCur;
      KeyPointVec kpCur;
      std::vector<cv::DMatch> matches;

      ExtractKeypoints(leftImage, kpCur, descCur);
      printf("Total Keypoints Left: %ld\n", kpCur.size());

      MatchKeypoints(descPrev, descCur, matches);
      printf("Total Matches to Keyframe: %ld\n", matches.size());

      GridSampleKeypoints(kpPrev, matches);
      printf("Total Matches After Grid Sampling: %ld\n", matches.size());

      cv::Mat descCurRight;
      KeyPointVec kpCurRight;
      std::vector<cv::DMatch> matchesStereo;
      ExtractKeypoints(rightImage, kpCurRight, descCurRight);
      printf("Total Keypoints Right: %ld\n", kpCurRight.size());

      MatchKeypoints(descCur, descCurRight, matchesStereo);
      printf("Total Stereo Matches Left: %ld\n", matchesStereo.size());


      printf("Total Matches Before Distance Filter: %ld\n", matches.size());
      FilterMatchesByDistance(matches, kpPrev, kpCur, 150.0f);
      printf("Total Matches After Distance Filter: %ld\n", matches.size());

      std::vector<int> keypointIDs(kpCur.size(), -1);
      TransferKeypointIDs(lastKeyframe.keypointIDs, keypointIDs, matches);
      printf("Total Keypoints: %ld\n", keypointIDs.size());

      Point2fVec pointsTrain;
      Point2fVec pointsQuery;
      ExtractMatchesAsPoints(kpPrev, kpCur, matches, pointsTrain, pointsQuery);
      printf("Total Motion Correspondences: %ld\n", pointsTrain.size());

      cv::Mat mask;
      cv::Mat pose = EstimateMotion(pointsTrain, pointsQuery, mask, leftCamera);
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
      TriangulateStereoNormal(kpCur, kpCurRight, matchesStereo, keypointIDs, _curPoints3D);
      printf("Total Landmarks Triangulated: %ld\n", _curPoints3D.size());

      std::cout << "Pose:" << std::endl << eigenPose << std::endl;
      std::cout << "Determinant: " << eigenPose.determinant() << std::endl;

      lastKeyframeImage = leftImage.clone();
      InsertKeyframe(eigenPose, descCur, kpCur, keypointIDs);
      
      InsertLandmarks(_curPoints3D);
      
      PrintKeyframeIDs();

      // Visualization and Logging Only --------------------------------------------

      std::cout << "Camera Pose: " << std::endl << pose << std::endl;

      cv::Mat outImage;
      OpenCVTools::DrawMatchesDouble(lastKeyframeImage, kpPrev, leftImage, kpCur, matches, outImage);
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


void VisualOdometry::MatchKeypoints(cv::Mat& descTrain, cv::Mat& descQuery, std::vector<cv::DMatch>& matches)
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
   //   std::vector<cv::DMatch> finalMatches;
   //   for (size_t i = 0; i < matches.size(); i++)
   //   {
   //         if(matches[i].distance < 40)
   //            finalMatches.push_back(matches[i]);
   //   }
   //   matches = finalMatches

   //   printf("MatchKeypoints(): Total Matches: {}", matches.size());
}

void VisualOdometry::TransferKeypointIDs(const std::vector<int>& trainIDs, std::vector<int>& queryIDs, std::vector<cv::DMatch>& matches)
{
   for(uint32_t i = 0; i<matches.size(); i++)
   {
      if(trainIDs[matches[i].trainIdx] != -1)
      {
         queryIDs[matches[i].queryIdx] = trainIDs[matches[i].trainIdx];
      }
   }
}

void VisualOdometry::GridSampleKeypoints(KeyPointVec& keypoints, std::vector<cv::DMatch>& matches)
{
   uint32_t xStep = width / xGridCount;
   uint32_t yStep = height / yGridCount;

   bool grid[yGridCount][xGridCount];
   memset(grid, false, sizeof(bool) * yGridCount * xGridCount);

   std::vector<cv::DMatch> finalMatches;
   for (uint32_t i = 0; i < matches.size(); i++)
   {
      cv::Point2f point(keypoints[matches[i].trainIdx].pt);

      if (point.x >= 0 && point.x < width && point.y >= 0 && point.y < height)
      {
         uint32_t xIndex = (uint32_t) ((float) point.x / (float) xStep);
         uint32_t yIndex = (uint32_t) ((float) point.y / (float) yStep);
         // printf("i: %d, Size: %ld Dims:%d %d Point: %.3lf %.3lf -> %d %d\n", i, matches.size(), width, height, point.x, point.y, xIndex, yIndex);

         if (xIndex < xGridCount && yIndex < yGridCount)
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



void VisualOdometry::TriangulateStereoNormal(KeyPointVec& pointsTrain, KeyPointVec& pointsQuery, std::vector<cv::DMatch>& matches,
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

         Z = leftCamera._fx * baselineDistance / (x1 - x2);
         X = x1 * baselineDistance / (x1 - x2);
         Y = y_hat * (baselineDistance / (x1 - x2));

         //         printf("Point3D: {} {} {}", X, Y, Z);
         if (Z > 0)
         {
            Eigen::Vector3f point(X, Y, Z);
            Eigen::Vector2f measurement(pointsTrain[match.trainIdx].pt.x, pointsTrain[match.trainIdx].pt.y);

            points3D.push_back({point, measurement, kpIDs[match.trainIdx]});
         }
      }
   }
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
                        useExtrinsicGuess, iterationsCount, reprojectionError, confidence, mask);
   cv::Rodrigues(rvec, rotation);

   cv::Mat cvPose = cv::Mat::eye(4, 4, CV_32FC1);
   rotation.copyTo(cvPose(cv::Range(0, 3), cv::Range(0, 3))); /* Excludes the 'end' element */
   tvec.copyTo(cvPose(cv::Range(0, 3), cv::Range(3, 4)));

   cv::invert(cvPose, cvPose);
   this->cvCurPose = (this->cvCurPose * cvPose);

   std::cout << "Pose: " << std::endl << cvPose << std::endl;

   // printf("%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\n", cvPose.at<float>(0), cvPose.at<float>(1), cvPose.at<float>(2),
   //       cvPose.at<float>(3), cvPose.at<float>(4), cvPose.at<float>(5), cvPose.at<float>(6), cvPose.at<float>(7), cvPose.at<float>(8), cvPose.at<float>(9),
   //       cvPose.at<float>(10), cvPose.at<float>(11));

   return cvPose;
}

void VisualOdometry::FilterMatchesByDistance(std::vector<cv::DMatch>& matches, const KeyPointVec& kpTrain, const KeyPointVec& kpQuery, float distanceThreshold)
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
   if (!prevFinalDisplay.empty() && prevFinalDisplay.rows != 0 && prevFinalDisplay.cols != 0)
      cv::imshow("Previous Keypoints Visualizer", prevFinalDisplay);
   if (!curFinalDisplay.empty() && curFinalDisplay.rows != 0 && curFinalDisplay.cols != 0)
      cv::imshow("Keypoints Visualizer", curFinalDisplay);
   cv::waitKey(delay);
}



// TODO: Move to OpenCV tools class
cv::Mat VisualOdometry::CalculateStereoDepth(cv::Mat left, cv::Mat right)
{
   cv::Mat disparity;
   stereo->compute(left, right, disparity);
   disparity.convertTo(disparity, CV_8U, 1.0);
   return disparity;
}

void VisualOdometry::Display(cv::Mat& image)
{
   curFinalDisplay = image.clone();
}

void VisualOdometry::ExtractMatchesAsPoints(const KeyPointVec& kpTrain, const KeyPointVec& kpQuery, const std::vector<cv::DMatch>& matches, Point2fVec& pointsTrain, Point2fVec& pointsQuery)
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

void VisualOdometry::Initialize(cv::Mat& leftImageCur, cv::Mat& rightImageCur)
{
   printf("Initialize()\n");

   printf("Extracting Keypoints Left\n");
   ExtractKeypoints(leftImageCur, kp_curLeft, desc_curLeft);

   printf("Extracting Keypoints Right\n");
   ExtractKeypoints(rightImageCur, kp_curRight, desc_curRight);

   printf("Matching Stereo Keypoints\n");
   MatchKeypoints(desc_curLeft, desc_curRight, curMatchesStereo);

   printf("Assigning Unique Labels: %ld\n", kp_curLeft.size());
   std::vector<int> kpIDs(kp_curLeft.size(), -1);
   for(int i = 0; i<(int)kp_curLeft.size(); i++)
   {
      uniqueKeypointID++;
      kpIDs[i] = uniqueKeypointID;
   }

   printf("Triangulating Stereo Keypoints\n");
   _curPoints3D.clear();
   TriangulateStereoNormal(kp_curLeft, kp_curRight, curMatchesStereo, kpIDs, _curPoints3D);

   printf("Inserting Keyframe Initial\n");
   InsertKeyframe(Eigen::Matrix4d::Identity(), desc_curLeft, kp_curLeft, kpIDs);
   PrintKeyframeIDs();

   _initialized = true;
}


void VisualOdometry::TrackCameraPose(const KeyPointVec& kp, const cv::Mat& desc, const PointLandmarkVec& landmarks)
{

   // Create a list of possible landmark IDs to project onto the camera view

   // Extract all landmark points that fall on the current camera view

      // Project 3D points onto 

   // 
   
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