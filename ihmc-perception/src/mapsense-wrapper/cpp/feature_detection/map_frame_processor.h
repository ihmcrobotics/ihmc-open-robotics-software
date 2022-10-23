#pragma once

#include "map_frame.h"
#include "planar_region.h"
#include <opencv2/highgui.hpp>
#include "application_state.h"

class MapFrameProcessor
{
   public:
      MapFrameProcessor(ApplicationState& app);

      void Init(ApplicationState& app);

      void GenerateSegmentation(MapFrame frame, std::vector<std::shared_ptr<PlanarRegion>>& planarRegionList);

      void DFS(uint16_t x, uint16_t y, uint8_t component, int& num, cv::Mat& debug, std::shared_ptr<PlanarRegion> planarRegion);

      void BoundaryDFS(int x, int y, int regionId, int component, int& num, cv::Mat& debug, std::shared_ptr<RegionRing> regionRing);

      void FindBoundaryAndHoles(std::vector<std::shared_ptr<PlanarRegion>>& planarRegionList);

      void printPatchGraph();

      void DisplayDebugger(int delay);

      void GrowRegionBoundary(std::vector<std::shared_ptr<PlanarRegion>>& regions);

      const cv::Mat& GetDebugMat() const { return _debugMat; };

      int adx[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
      int ady[8] = {-1, -1, -1, 0, 1, 1, 1, 0};

   public:
      MapFrame _frame;
      cv::Mat _debugMat;
      Eigen::MatrixXi _visited;
      Eigen::MatrixXi _boundary;
      Eigen::MatrixXi _region;
      ApplicationState& _app;
      Eigen::Vector2i _connect[8] = {Eigen::Vector2i(-1,-1), Eigen::Vector2i(1,0),Eigen::Vector2i(2,0),Eigen::Vector2i(0,1),Eigen::Vector2i(2,1),Eigen::Vector2i(0,2),Eigen::Vector2i(1,2), Eigen::Vector2i(2,2)};

      int SUB_H = 0;
      int SUB_W = 0;
      int PATCH_HEIGHT = 0;
      int PATCH_WIDTH = 0;
      int INPUT_HEIGHT = 0;
      int INPUT_WIDTH = 0;

};

