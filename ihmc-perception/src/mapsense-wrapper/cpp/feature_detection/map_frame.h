#pragma once

#include "opencv2/core/core.hpp"
#include <opencv2/imgproc.hpp>
#include "application_state.h"

class MapFrame
{
   public:

      cv::Mat regionOutput;
      cv::Mat patchData;

      void setRegionOutput(cv::Mat& regionOutput);

      void setPatchData(cv::Mat& patchData);

      cv::Mat& getRegionOutput();

      cv::Mat& getPatchData();

      void drawGraph(cv::Mat& img, ApplicationState app);

      cv::Vec6f getPatch(int x, int y);
};

