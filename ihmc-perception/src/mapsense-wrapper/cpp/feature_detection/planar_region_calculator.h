#pragma once

#include "opencv4/opencv2/highgui/highgui.hpp"
#include "opencv4/opencv2/core/core.hpp"
#include <CL/cl.h>

#include "map_frame_processor.h"
#include "opencl_manager.h"

#include "geom_tools.h"
#include "point_cloud.h"

#include "application_state.h"

class MapFrame;
class PlanarRegion;


class PlanarRegionCalculator
{
   public:
      PlanarRegionCalculator(ApplicationState& app);

      void GenerateRegionsFromPointCloud(ApplicationState& appState, std::vector<float>& vertices);

      bool GeneratePatchGraphFromDepth(ApplicationState& appState);

      void generateRegionsFromDepth(ApplicationState& appState, cv::Mat& depth, double inputTimestamp);

      void LoadRegions(std::string path, std::vector<std::string>& fileNames, int index);

      void getFilteredDepth(cv::Mat& dispDepth, ApplicationState& appState);

      void setOpenCLManager(OpenCLManager* ocl) {_openCL = ocl;}

      uint8_t CreateParameterBuffer(const ApplicationState& app);



   private:

      bool _render = true;

      int depthFileSelected = 0;
      int cloudFileSelected = 0;
      int frameId = 0;
      int depthReceiverId = -1;

      float xAngle = 0;
      float yAngle = 0;
      float zAngle = 0;
      float xOffset = 0;
      float yOffset = 0;
      float zOffset = 0;

      double inputTimestamp;

      ApplicationState& app;
      OpenCLManager* _openCL;

      std::vector<std::shared_ptr<PlanarRegion>> planarRegionList;
      std::vector<std::shared_ptr<PlanarRegion>> _hashRegionsZUp;
      std::vector<std::shared_ptr<PlanarRegion>> _depthRegionsZUp;
      std::unordered_map<std::string, cv::Mat> channelMap;
      std::vector<std::string> depthFiles, cloudFiles;

      cv::Mat inputDepth;
      cv::Mat inputColor;
      cv::Mat filteredDepth;
      cv::Mat inputStereoLeft, inputStereoRight;

      MapFrame output;
      MapFrameProcessor* _depthMapFrameProcessor;
      MapFrameProcessor* _hashMapFrameProcessor;

      RigidBodyTransform _headToL515Transform;
      RigidBodyTransform _headToOusterTransform;
      RigidBodyTransform _transformZUp;

};

