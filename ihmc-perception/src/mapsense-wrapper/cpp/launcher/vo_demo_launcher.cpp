#include "vo_demo_launcher.h"

#include "file_tools.h"

#include "opencv4/opencv2/core.hpp"

VODemoLauncher::VODemoLauncher()
{
   VisualOdometry vo(appState);

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
      TestEstimateMotion(vo, fileNames, 0, 10);
   }
}

void VODemoLauncher::Run(VisualOdometry& vo, const std::vector<std::string>& fileNames, int index)
{
   cv::Mat leftImage = cv::imread(leftDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);
   cv::Mat rightImage = cv::imread(rightDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);

   vo.UpdateStereo(leftImage, rightImage);

   vo.Show();

}

void VODemoLauncher::TestExtractKeypoints(VisualOdometry& vo, const std::vector<std::string>& fileNames, int index)
{
   cv::Mat leftImage = cv::imread(leftDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);
   cv::Mat rightImage = cv::imread(rightDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);

   cv::Mat descriptors;
   std::vector<cv::KeyPoint> keypoints;
   
   vo.ExtractKeypoints(leftImage, keypoints, descriptors);

   vo.DrawKeypoints(leftImage, keypoints);
   vo.Show(0);
}

void VODemoLauncher::TestMatchKeypoints(VisualOdometry& vo, const std::vector<std::string>& fileNames, int index)
{
   cv::Mat leftImage = cv::imread(leftDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);
   cv::Mat rightImage = cv::imread(rightDatasetDirectory + fileNames[index], cv::IMREAD_GRAYSCALE);

   cv::Mat leftDescriptors;
   cv::Mat rightDescriptors;
   std::vector<cv::KeyPoint> leftKeypoints;
   std::vector<cv::KeyPoint> rightKeypoints;
   
   std::vector<cv::DMatch> matches;

   vo.ExtractKeypoints(leftImage, leftKeypoints, leftDescriptors);
   vo.ExtractKeypoints(leftImage, rightKeypoints, rightDescriptors);

   vo.MatchKeypoints(leftDescriptors, rightDescriptors, matches);

   vo.DrawKeypointMatches(leftImage, leftKeypoints, rightImage, rightKeypoints, matches);
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

void VODemoLauncher::TestEstimateMotion(VisualOdometry& vo, const std::vector<std::string>& fileNames, int indexOne, int indexTwo)
{
   cv::Mat leftImagePrev = cv::imread(leftDatasetDirectory + fileNames[indexOne], cv::IMREAD_GRAYSCALE);
   cv::Mat rightImagePrev = cv::imread(rightDatasetDirectory + fileNames[indexOne], cv::IMREAD_GRAYSCALE);

   cv::Mat leftImageCur = cv::imread(leftDatasetDirectory + fileNames[indexTwo], cv::IMREAD_GRAYSCALE);
   cv::Mat rightImageCur = cv::imread(rightDatasetDirectory + fileNames[indexTwo], cv::IMREAD_GRAYSCALE);

   cv::Mat disparityPrev = vo.CalculateStereoDepth(leftImagePrev, rightImagePrev);
   cv::Mat disparityCur = vo.CalculateStereoDepth(leftImageCur, rightImageCur);

   vo.Display(disparityPrev);
   vo.Show(0);

   vo.Display(disparityCur);
   vo.Show(0);
}


int main()
{
   VODemoLauncher demo;
}

