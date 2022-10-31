#pragma once

#include "visual_odometry_external.h"

class VODemoLauncher
{
   public:

      VODemoLauncher();

      void Run(VisualOdometry& vo, const std::vector<std::string>& fileNames, int index);

      void TestExtractKeypoints(VisualOdometry& vo, const std::vector<std::string>& fileNames, int index);
      void TestMatchKeypoints(VisualOdometry& vo, const std::vector<std::string>& fileNames, int index);
      void TestStereoDisparityCalculation(VisualOdometry& vo, const std::vector<std::string>& fileNames, int index);
      void TestMatchKeypointsMonocular(VisualOdometry& vo, const std::vector<std::string>& fileNames, int indexOne, int indexTwo);
      void TestEstimateMotion(VisualOdometry& vo, const std::vector<std::string>& fileNames, int indexOne, int indexTwo);

   private:

      ApplicationState appState;

      bool loop = false;

      std::string leftDatasetDirectory = "/home/quantum/Workspace/Data/Datasets/sequences/00/image_0/";
      std::string rightDatasetDirectory = "/home/quantum/Workspace/Data/Datasets/sequences/00/image_1/";
};

