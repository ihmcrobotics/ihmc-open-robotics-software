#include "vo_demo_launcher.h"

#include "file_tools.h"
#include "opencv_tools.h"

#include "opencv4/opencv2/core.hpp"

VODemoLauncher::VODemoLauncher()
{
   VisualOdometry vo(appState);
   FeatureExtractor extractor(1000);

   std::vector<std::string> fileNames;

   FileTools::getFileNames(leftDatasetDirectory, fileNames);

   if(loop)
   {
      for(uint32_t i = 0; i<fileNames.size(); i++)
      {
         Run(vo, fileNames, i);
      }

   }
   else
   {
      TestEstimateMotion(vo, extractor, fileNames, 0, 1);
   }
}

void VODemoLauncher::Run(VisualOdometry& vo, const std::vector<std::string>& fileNames, int index)
{
   cv::Mat leftImage = cv::imread(leftDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);
   cv::Mat rightImage = cv::imread(rightDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);



   // vo.UpdateStereo(leftImage, rightImage);

   vo.Show();

}

void VODemoLauncher::ExtractKeypoints(VisualOdometry& vo, FeatureExtractor& extractor, const std::vector<std::string>& fileNames, int index)
{
   cv::Mat leftImage = cv::imread(leftDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);
   cv::Mat rightImage = cv::imread(rightDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);

   cv::Mat descriptors;
   std::vector<cv::KeyPoint> keypoints;
   
   extractor.ExtractKeypoints(leftImage, keypoints, descriptors);

   cv::Mat outImage;
   cv::drawKeypoints(leftImage, keypoints, outImage);
   vo.Display(outImage);
   vo.Show(0);
}

void VODemoLauncher::TestMatchKeypoints(VisualOdometry& vo, FeatureExtractor& extractor, const std::vector<std::string>& fileNames, int index)
{
   cv::Mat leftImage = cv::imread(leftDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);
   cv::Mat rightImage = cv::imread(rightDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);

   cv::Mat leftDescriptors;
   cv::Mat rightDescriptors;
   std::vector<cv::KeyPoint> leftKeypoints;
   std::vector<cv::KeyPoint> rightKeypoints;
   
   std::vector<cv::DMatch> matches;

   extractor.ExtractKeypoints(leftImage, leftKeypoints, leftDescriptors);
   extractor.ExtractKeypoints(rightImage, rightKeypoints, rightDescriptors);

   extractor.MatchKeypoints(leftDescriptors, rightDescriptors, matches);

   cv::Mat outImage;
   cv::drawMatches(leftImage, leftKeypoints, rightImage, rightKeypoints, matches, outImage);
   vo.Display(outImage);
   vo.Show(0);
}


void VODemoLauncher::TestStereoDisparityCalculation(VisualOdometry& vo, const std::vector<std::string>& fileNames, int index)
{
   cv::Mat leftImage = cv::imread(leftDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);
   cv::Mat rightImage = cv::imread(rightDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);

   cv::Mat disparity = vo.CalculateStereoDepth(leftImage, rightImage);

   vo.Display(disparity);
   vo.Show(0);
}

void VODemoLauncher::TestStereoTriangulation(VisualOdometry& vo, FeatureExtractor& extractor, const std::vector<std::string>& fileNames, int index)
{
   cv::Mat leftImage = cv::imread(leftDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);
   cv::Mat rightImage = cv::imread(rightDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);
   
   cv::Mat descCur;
   cv::Mat depth;
   std::vector<cv::KeyPoint> kpCur;
   std::vector<Eigen::Vector3f> points;

   cv::Mat disparity = vo.CalculateStereoDepth(leftImage, rightImage);

   extractor.ExtractKeypoints(leftImage, kpCur, descCur);

   OpenCVTools::ConvertDisparityToDepth(disparity, depth);

   vo.TriangulateKeypointsByDisparity(kpCur, depth, points);

   OpenCVTools::DisplayImage("TestStereoTriangulation", disparity, 0);
}

void VODemoLauncher::TestMatchKeypointsMonocular(VisualOdometry& vo, FeatureExtractor& extractor, const std::vector<std::string>& fileNames, int indexOne, int indexTwo)
{
   cv::Mat leftImagePrev = cv::imread(leftDatasetDirectory + fileNames[indexOne], cv::IMREAD_GRAYSCALE);
   cv::Mat leftImageCur = cv::imread(leftDatasetDirectory + fileNames[indexTwo], cv::IMREAD_GRAYSCALE);

   cv::Mat descPrev;
   cv::Mat descCur;
   std::vector<cv::KeyPoint> kpPrev;
   std::vector<cv::KeyPoint> kpCur;
   
   std::vector<cv::DMatch> matches;

   extractor.ExtractKeypoints(leftImagePrev, kpPrev, descPrev);
   extractor.ExtractKeypoints(leftImageCur, kpCur, descCur);

   extractor.MatchKeypoints(descPrev, descCur, matches);

   printf("Total Matches Before: %ld\n", matches.size());
   vo.FilterMatchesByDistance(matches, kpPrev, kpCur, 100.0f);
   printf("Total Matches After: %ld\n", matches.size());


   cv::Mat outImage;
   OpenCVTools::DrawMatchesDouble(leftImagePrev, kpPrev, leftImageCur, kpCur, matches, outImage);
   OpenCVTools::DisplayImage("TestMatchKeypointsMonocular", outImage, 0);
}

void VODemoLauncher::TestEstimateMotion(VisualOdometry& vo, FeatureExtractor& extractor, const std::vector<std::string>& fileNames, int indexOne, int indexTwo)
{
   cv::Mat leftImagePrev = cv::imread(leftDatasetDirectory + fileNames[indexOne], cv::IMREAD_GRAYSCALE);
   cv::Mat leftImageCur = cv::imread(leftDatasetDirectory + fileNames[indexTwo], cv::IMREAD_GRAYSCALE);

   cv::Mat descPrev;
   cv::Mat descCur;
   std::vector<cv::KeyPoint> kpPrev;
   std::vector<cv::KeyPoint> kpCur;
   
   std::vector<cv::DMatch> matches;

   extractor.ExtractKeypoints(leftImagePrev, kpPrev, descPrev);
   extractor.ExtractKeypoints(leftImageCur, kpCur, descCur);

   extractor.MatchKeypoints(descPrev, descCur, matches);

   printf("Total Matches Before: %ld\n", matches.size());
   vo.FilterMatchesByDistance(matches, kpPrev, kpCur, 100.0f);
   printf("Total Matches After: %ld\n", matches.size());


   CameraModel cam;
   cam.SetParams(718.856, 718.856, 607.193, 185.216);

   std::vector<cv::Point2f> pointsTrain;
   std::vector<cv::Point2f> pointsQuery;

   vo.ExtractMatchesAsPoints(kpPrev, kpCur, matches, pointsTrain, pointsQuery);

   cv::Mat mask;
   cv::Mat pose = vo.EstimateMotion(pointsTrain, pointsQuery, mask, cam);

   std::cout << "Camera Pose: " << std::endl << pose << std::endl;


   cv::Mat outImage;
   OpenCVTools::DrawMatchesDouble(leftImagePrev, kpPrev, leftImageCur, kpCur, matches, outImage);
   OpenCVTools::DisplayImage("TestMatchKeypointsMonocular", outImage, 0);
}

int main()
{
   VODemoLauncher demo;
}

