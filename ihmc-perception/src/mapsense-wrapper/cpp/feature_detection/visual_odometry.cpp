#include "visual_odometry.h"
#include "opencv_tools.h"

VisualOdometry::VisualOdometry(ApplicationState& app) : _appState(app)
{
   leftCamera.SetParams(718.856, 718.856, 607.193, 185.216);

   cameraPose = Eigen::Matrix4f::Identity();



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
      MatchKeypoints(descPrev, descCur, matches);

      printf("Total Matches Before: %ld\n", matches.size());
      FilterMatchesByDistance(matches, kpPrev, kpCur, 100.0f);
      printf("Total Matches After: %ld\n", matches.size());


      Point2fVec pointsTrain;
      Point2fVec pointsQuery;
      ExtractMatchesAsPoints(kpPrev, kpCur, matches, pointsTrain, pointsQuery);

      cv::Mat mask;
      cv::Mat pose = EstimateMotion(pointsTrain, pointsQuery, mask, leftCamera);

      cv::Mat cvPose = pose;
      Eigen::Map<Eigen::Matrix<float, 4, 4>, Eigen::RowMajor> eigenPose(reinterpret_cast<float *>(cvPose.data));
      eigenPose.transposeInPlace();
      cameraPose = cameraPose * eigenPose;

      InsertKeyframe(cameraPose, descCur, kpCur, leftImage);


      // Visualization and Logging Only --------------------------------------------


      std::cout << "Camera Pose: " << std::endl << pose << std::endl;

      cv::Mat outImage;
      OpenCVTools::DrawMatchesDouble(lastKeyframe.leftImage, kpPrev, leftImage, kpCur, matches, outImage);
      OpenCVTools::DisplayImage("TestMatchKeypointsMonocular", outImage, 1);

      // Visualization and Logging Only --------------------------------------------
   }
   printf("Total Keyframes: %ld\n", _keyframes.size());

   return true;
}

// bool VisualOdometry::UpdateStereoOld(const cv::Mat& leftImage, const cv::Mat& rightImage)
// {
//    auto start_point = std::chrono::steady_clock::now();
//    cv::Mat cvPose, points4D;
//    PointLandmarkVec points3D;

//    if (!leftImage.empty() && leftImage.rows > 0 && leftImage.cols > 0 && !rightImage.empty() && rightImage.rows > 0 && rightImage.cols > 0)
//    {
//       /* During first iteration, simply store (prev) grayscale images, extract keypoints,
//        * and insert the first keyframe at identity pose, and return false, since no pose was computed. */
//       if (count == 0)
//       {
//          width = (uint32_t)leftImage.cols;
//          height = (uint32_t)leftImage.rows;
//          prevLeft = leftImage.clone();
//          prevRight = rightImage.clone();
//          ExtractKeypoints(prevLeft, kp_prevLeft, desc_prevLeft);
//          ExtractKeypoints(prevRight, kp_prevRight, desc_prevRight);
//          _keyframes.emplace_back(Keyframe(Eigen::Matrix4f::Identity(), desc_prevLeft.clone(), desc_prevRight.clone(), kp_prevLeft, kp_prevRight, leftImage.clone(), rightImage.clone()));
//          count++;
//          return false;
//       }

//       /* From second iteration onwards, store (cur) grayscale images, initialize, and insert keyframes when needed. */
//       curLeft = leftImage.clone();
//       curRight = rightImage.clone();

//       /* Initialization step: Build initial local map of 3D points; Insert the second keyframe. */
//       if (!_initialized)
//       {
//          auto kf = _keyframes[0];
//          CalculateOdometry_ORB(kf, curLeft, curRight, cvPose, points3D);

//          Eigen::Map<Eigen::Matrix<float, 4, 4>, Eigen::RowMajor> eigenPose(reinterpret_cast<float *>(cvPose.data));
//          eigenPose.transposeInPlace();
//          cameraPose = cameraPose * eigenPose;

//         prevFinalDisplay = leftImage.clone();

//         OpenCVTools::DrawMatchesSingle(prevPoints2D, curPoints2D, prevFinalDisplay);

//          if (cameraPose.block<3, 1>(0, 3).norm() > 1)
//          {
//             _initialized = true;
//             _keyframes.emplace_back(Keyframe(cameraPose, desc_curLeft.clone(), desc_curRight.clone(),
//                                              kp_curLeft, kp_curRight, leftImage.clone(), rightImage.clone()));
//          }
//       } else
//       {

//          auto kf = _keyframes[_keyframes.size() - 1];
//          CalculateOdometry_ORB(kf, curLeft, curRight, cvPose, points3D);
//          cvCurPose = cvCurPose * cvPose;

//          Eigen::Map<Eigen::Matrix<float, 4, 4>, Eigen::RowMajor> eigenPose(reinterpret_cast<float *>(cvPose.data));
//          eigenPose.transposeInPlace();
//          cameraPose = cameraPose * eigenPose;

//          prevFinalDisplay = leftImage.clone();
//          OpenCVTools::DrawMatchesSingle(prevPoints2D, curPoints2D, prevFinalDisplay);

//          if (eigenPose.block<3, 1>(0, 3).norm() > 0.8)
//          {
//             _initialized = true;
//             _keyframes.emplace_back(
//                   Keyframe(cameraPose, desc_curLeft.clone(), desc_curRight.clone(), kp_curLeft, kp_curRight, leftImage.clone(), rightImage.clone()));

//             printf("Performing Bundle Adjustment.");

//             /* ------------------------- BUNDLE ADJUSTMENT ------------------------------*/
// //            std::vector<Eigen::Matrix4f> poses;
// //            poses.emplace_back(_keyframes[_keyframes.size() - 2].pose);
// //            poses.emplace_back(_keyframes[_keyframes.size() - 1].pose);
// //            _bundleAdjustment->Update(points3D, poses);
//             /* ------------------------- BUNDLE ADJUSTMENT ------------------------------*/

// //            _bundleAdjustment->Optimize();


//             // if (axes)
//             // {
//             //    glm::mat4 glmTransform;
//             //    for (uint32_t i = 0; i < 4; ++i)
//             //       for (int j = 0; j < 4; ++j)
//             //          glmTransform[j][i] = cameraPose(i, j);
//             //    glmTransform[3][0] *= scalar;
//             //    glmTransform[3][1] *= scalar;
//             //    glmTransform[3][2] *= scalar;
//             //    axes->ApplyTransform(glmTransform);
//             // }

//             /* Triangulated Points */
//             /* TODO: Filter points by 5-point algorithm mask before triangulation. */
//             // if (cloud)
//             // {
//             //    for (uint32_t i = 0; i < points3D.size(); i++)
//             //    {
//             //       if (i % 4 == 0)
//             //       {
//             //          Eigen::Vector4f point;
//             //          point << points3D[i]._point3D, 1;
//             //          point = cameraPose * point;
//             //          cloud->InsertVertex(scalar * point.x() / point.w(), scalar * point.y() / point.w(), scalar * point.z() / point.w());
//             //       }
//             //    }
//             // }
//          }
//       }
//    }

//    auto end_point = std::chrono::steady_clock::now();
//    long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
//    long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();

//    float duration = (end - start) * 0.001f;

//    printf("(Visual Odometry) Total Time Taken: %.3lf ms\n", duration);

//    return true;
// }


void VisualOdometry::DrawLandmarks(cv::Mat& img, PointLandmarkVec& landmarks)
{
   for (uint32_t i = 0; i < landmarks.size(); i++)
   {
      cv::Point2f first(landmarks[i].GetMeasurements2D()[0].x() + leftCamera._cx, landmarks[i].GetMeasurements2D()[0].y() + leftCamera._cy);
      cv::Point2f second(landmarks[i].GetMeasurements2D()[1].x() + leftCamera._cx, landmarks[i].GetMeasurements2D()[1].y() + leftCamera._cy);

      float dist = cv::norm(first - second);
//      printf("DrawLandmark: First({}, {}), Second:({} {}), Dist({}), Total Measurements: {}", first.x, first.y, second.x, second.y, dist, landmarks[i].GetMeasurements2D().size());

      if(dist < 100)
      {
         line(img, first, second, cv::Scalar(0, 255, 0), 3);
         circle(img, first, 2, cv::Scalar(0, 0, 0), -1);
         circle(img, second, 2, cv::Scalar(255, 255, 255), -1);
      }

   }
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

void VisualOdometry::GridSampleKeypoints(KeyPointVec& keypoints, std::vector<cv::DMatch>& matches)
{
   uint32_t xStep = width / xGridCount;
   uint32_t yStep = height / yGridCount;

   bool grid[yGridCount][xGridCount];
   memset(grid, false, sizeof(bool) * yGridCount * xGridCount);

   std::vector<cv::DMatch> finalKeypoints;
   for (uint32_t i = 0; i < matches.size(); i++)
   {
      cv::Point2f point(keypoints[matches[i].trainIdx].pt);

      if (point.x >= 0 && point.x < width && point.y >= 0 && point.y < height)
      {
         uint32_t xIndex = (uint32_t) ((float) point.x / (float) xStep);
         uint32_t yIndex = (uint32_t) ((float) point.y / (float) yStep);
         //   printf("i: {}, Size: {} Dims:{} {} Point: {} {} -> {} {}", i, matches.size(), width, height, point.x, point.y, xIndex, yIndex);

         if (xIndex < xGridCount && yIndex < yGridCount)
         {
            if (!grid[yIndex][xIndex])
            {
               finalKeypoints.push_back(matches[i]);
               grid[yIndex][xIndex] = true;
            }
         }
      }
   }

   matches.clear();
   matches = finalKeypoints;
}



void VisualOdometry::TriangulateStereoNormal(KeyPointVec& pointsTrain, KeyPointVec& pointsQuery, std::vector<cv::DMatch>& matches,
                                             PointLandmarkVec& points3D)
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

            PointLandmark landmark(point);
            Eigen::Vector2f measurement(x1, y1);
            landmark.AddMeasurement2D(measurement, match.trainIdx, 0);

            points3D.emplace_back(landmark);
         }
      }
   }
   //   printf("Triangulate(): Total Depth Points: {}", points3D.size());
}

void VisualOdometry::ExtractFinalSet(std::vector<cv::DMatch> leftMatches, KeyPointVec curLeftKp, PointLandmarkVec& points3D)
{
   for (auto match: leftMatches)
   {
      for (uint32_t i = 0; i < points3D.size(); i++)
      {
         if (points3D[i]._index[0] == match.trainIdx)
         {
            Eigen::Vector2f measurement(curLeftKp[match.queryIdx].pt.x - leftCamera._cx,
                            curLeftKp[match.queryIdx].pt.y - leftCamera._cy);

            // Eigen::Vector2f prevMeasurement = points3D[i].GetMeasurements2D()[0];
            // Eigen::Vector2f oneMeasurement = points3D[i].GetMeasurements2D()[1];

//            printf("Match: PrevStereoPoint({}, {})", measurement.x(), measurement.y());
//            printf("Match: Zero({}, {}), One:({} {})", prevMeasurement.x(), prevMeasurement.y(), oneMeasurement.x(), oneMeasurement.y());

            points3D[i].AddMeasurement2D(measurement, match.queryIdx, 1);
         }
      }

   }

   for(int i = points3D.size() - 1; i>=0; i--)
   {
      if(points3D[i].GetMeasurements2D().size() != 2)
      {
         points3D.erase(points3D.begin() + i);
      }
   }
   printf("Total Overlap PointLandmarks: %ld\n", points3D.size());
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

// void VisualOdometry::CalculateOdometry_ORB(Keyframe& kf, cv::Mat leftImage, cv::Mat rightImage, cv::Mat& cvPose, PointLandmarkVec& points3D)
// {
//    printf("CalculateOdometry_ORB\n");

//    printf("Extract Keypoints\n");
//    ExtractKeypoints(leftImage, kp_curLeft, desc_curLeft);
//    ExtractKeypoints(rightImage, kp_curRight, desc_curRight);

//    printf("Match Keypoints\n");
//    MatchKeypoints(kf.descLeft, desc_curLeft, matchesLeft);
//    MatchKeypoints(kf.descLeft, kf.descRight, prevMatchesStereo);

//    printf("Stereo Matches: %ld %d %d\n", prevMatchesStereo.size(), kf.descLeft.rows, kf.descRight.rows);
// //   cv::drawMatches(kf.rightImage, kf.keypointsRight, kf.leftImage, kf.keypointsLeft, prevMatchesStereo, prevFinalDisplay);

//    printf("Filter Matches Left\n");
//    FilterMatchesByDistance(matchesLeft,kf.keypointsLeft, kp_curLeft, 50.0f);

//    printf("Filter Matches Stereo\n");
//    FilterMatchesByDistance(prevMatchesStereo, kf.keypointsLeft, kp_curLeft, 8.0f);


//    printf("Points To Be Triangulated: %ld\n", prevMatchesStereo.size());
//    TriangulateStereoNormal(kf.keypointsLeft, kf.keypointsRight, prevMatchesStereo, points3D);
//    printf("Points Triangulated: %ld\n", points3D.size());

//    printf("Extract 2D Feature Points\n");
   

//    printf("Points for Motion Estimation: %ld %ld\n", prevPoints2D.size(), curPoints2D.size());

//    cv::Mat mask, pose;
//    if (prevPoints2D.size() >= 10 && curPoints2D.size() >= 10)
//    {
//       pose = EstimateMotion(prevPoints2D, curPoints2D, mask, leftCamera);
//       cvPose = pose;

//       /* Triangulate Feature Points */
// //      if (prevPoints2D.size() > 10)
// //      {
// //         cv::Mat points = TriangulatePoints(prevPoints2D, curPoints2D, leftCamera, pose);
// //         points4D = points;
// //      }
//    }

//    printf("Extract Final Set\n");
//    ExtractFinalSet(matchesLeft, kp_curLeft, points3D);

//    printf("Draw Landmarks\n");
//    DrawLandmarks(prevFinalDisplay, points3D);



// //   for(auto point : points3D)
// //   {
// //      printf("Landmark: {} {} {}", point.GetMeasurements2D()[0].y(), point.GetMeasurements2D()[1].y(), point.GetMeasurements2D()[2].y());
// //   }




//    //   printf("%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\n",
//    //          cvPose.at<float>(0), cvPose.at<float>(1), cvPose.at<float>(2), cvPose.at<float>(3),
//    //          cvPose.at<float>(4), cvPose.at<float>(5), cvPose.at<float>(6), cvPose.at<float>(7),
//    //          cvPose.at<float>(8), cvPose.at<float>(9), cvPose.at<float>(10), cvPose.at<float>(11));



//    //   cv::drawMatches(curLeft, kp_curLeft, kf.image, kf.keypoints, matchesLeft, prevFinalDisplay);

//    printf("Reset Buffers\n");
//    prevLeft = curLeft.clone();
//    desc_prevLeft = desc_curLeft.clone();
//    kp_prevLeft = kp_curLeft;

//    prevRight = curRight.clone();
//    desc_prevRight = desc_curRight.clone();
//    kp_prevRight = kp_curRight;

//    count++;

// //   prevFinalDisplay = leftImage;

//    printf("CalculateOdometry_ORB Finished\n");
// }

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


void VisualOdometry::UpdateStereoExternal(cv::Mat& leftImageCur, cv::Mat& rightImageCur)
{

   ExtractKeypoints(leftImageCur, kp_curLeft, desc_curLeft);
   ExtractKeypoints(rightImageCur, kp_curRight, desc_curRight);

   // MatchKeypoints(desc_curLeft, descCur, matches);

   // printf("Total Matches Before: %ld\n", matches.size());
   // FilterMatchesByDistance(matches, kpPrev, kpCur, 100.0f);
   // printf("Total Matches After: %ld\n", matches.size());


   // CameraModel cam;
   // cam.SetParams(718.856, 718.856, 607.193, 185.216);

   // Point2fVec pointsTrain;
   // Point2fVec pointsQuery;

   // ExtractMatchesAsPoints(kpPrev, kpCur, matches, pointsTrain, pointsQuery);

   // cv::Mat mask;
   // cv::Mat pose = EstimateMotion(pointsTrain, pointsQuery, mask, cam);

   // std::cout << "Camera Pose: " << std::endl << pose << std::endl;


   // cv::Mat outImage;
   // OpenCVTools::DrawMatchesDouble(leftImageCur, kpPrev, rightImageCur, kpCur, matches, outImage);
   // OpenCVTools::DisplayImage("TestMatchKeypointsMonocular", outImage, 0);
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
   ExtractKeypoints(leftImageCur, kp_curLeft, desc_curLeft);
   ExtractKeypoints(rightImageCur, kp_curRight, desc_curRight);

   MatchKeypoints(desc_curLeft, desc_curRight, curMatchesStereo);

   InsertKeyframe(Eigen::Matrix4f::Identity(), desc_curLeft, kp_curLeft, leftImageCur);


   _initialized = true;
}

void VisualOdometry::InsertKeyframe(Eigen::Matrix4f pose, const cv::Mat& descLeft, const cv::Mat& descRight, KeyPointVec& kpLeft, KeyPointVec& kpRight)
{
   _keyframes.emplace_back(Keyframe(_keyframes.size(), pose, descLeft.clone(), descRight.clone(), kpLeft, kpRight));
}

void VisualOdometry::InsertKeyframe(Eigen::Matrix4f pose, cv::Mat& descLeft, cv::Mat& descRight, KeyPointVec& kpLeft, KeyPointVec& kpRight, const cv::Mat& leftMat, const cv::Mat& rightMat)
{
   _keyframes.emplace_back(Keyframe(_keyframes.size(), pose, descLeft.clone(), descRight.clone(), kpLeft, kpRight, leftMat.clone(), rightMat.clone()));
}

void VisualOdometry::InsertKeyframe(Eigen::Matrix4f pose, cv::Mat& descLeft, KeyPointVec& kpLeft, const cv::Mat& leftMat)
{
   _keyframes.emplace_back(Keyframe(_keyframes.size(), pose, descLeft.clone(), kpLeft, leftMat.clone()));
}
