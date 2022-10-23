#include "map_frame.h"

void MapFrame::setRegionOutput(cv::Mat& regionOutput)
{
   this->regionOutput = regionOutput;
}

void MapFrame::setPatchData(cv::Mat& patchData)
{
   this->patchData = patchData;
}

cv::Mat& MapFrame::getRegionOutput()
{
   return regionOutput;
}

cv::Mat& MapFrame::getPatchData()
{
   return patchData;
}

void MapFrame::drawGraph(cv::Mat& img, ApplicationState app)
{
   for (int j = 0; j < app.SUB_H - 1; j++)
   {
      for (int i = 0; i < app.SUB_W - 1; i++)
      {
         if (patchData.at<uint8_t>(j, i) == 255)
         {
            line(img, cv::Point(i * app.DEPTH_PATCH_HEIGHT + app.DEPTH_PATCH_HEIGHT / 2, j * app.DEPTH_PATCH_WIDTH + app.DEPTH_PATCH_WIDTH / 2),
                 cv::Point((i + 1) * app.DEPTH_PATCH_HEIGHT + app.DEPTH_PATCH_HEIGHT / 2, j * app.DEPTH_PATCH_WIDTH + app.DEPTH_PATCH_WIDTH / 2), cv::Scalar(0, 150, 0), 1);
            line(img, cv::Point(i * app.DEPTH_PATCH_HEIGHT + app.DEPTH_PATCH_HEIGHT / 2, j * app.DEPTH_PATCH_WIDTH + app.DEPTH_PATCH_WIDTH / 2),
                 cv::Point(i * app.DEPTH_PATCH_HEIGHT + app.DEPTH_PATCH_HEIGHT / 2, (j + 1) * app.DEPTH_PATCH_WIDTH + app.DEPTH_PATCH_WIDTH / 2), cv::Scalar(0, 150, 0), 1);
            circle(img, cv::Point(i * app.DEPTH_PATCH_HEIGHT + app.DEPTH_PATCH_HEIGHT / 2, j * app.DEPTH_PATCH_WIDTH + app.DEPTH_PATCH_WIDTH / 2), 2, cv::Scalar(0, 200, 0), -1);
         }
      }
   }
}

cv::Vec6f MapFrame::getPatch(int x, int y)
{
   return cv::Vec6f(this->getRegionOutput().at<float>(x, y), this->getRegionOutput().at<float>(x, y), this->getRegionOutput().at<float>(x, y),
                this->getRegionOutput().at<float>(x, y), this->getRegionOutput().at<float>(x, y), this->getRegionOutput().at<float>(x, y));
}

