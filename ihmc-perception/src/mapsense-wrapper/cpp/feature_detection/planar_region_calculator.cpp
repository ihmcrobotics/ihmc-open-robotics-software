#include "planar_region_calculator.h"
#include "buffer_tools.h"

PlanarRegionCalculator::PlanarRegionCalculator(ApplicationState& app) : app(app)
{
   std::cout << "PlanarRegionCalculator Created" << std::endl;
   // MS_INFO("Creating PlanarRegionCalculator");
   _depthMapFrameProcessor = new MapFrameProcessor(app);
   _hashMapFrameProcessor = new MapFrameProcessor(app);

   app.REGION_MODE = 0;
   _depthMapFrameProcessor->Init(app);

   app.REGION_MODE = 1;
   _hashMapFrameProcessor->Init(app);

   inputDepth = cv::Mat(app.DEPTH_INPUT_HEIGHT, app.DEPTH_INPUT_WIDTH, CV_16UC1);
   inputColor = cv::Mat(app.DEPTH_INPUT_HEIGHT, app.DEPTH_INPUT_WIDTH, CV_8UC3);
   
   // origin[0] = 0;
   // origin[0] = 0;
   // origin[0] = 0;

   // AppUtils::getFileNames(ros::package::getPath("map_sense") + "/Extras/Clouds/", cloudFiles);

   _headToOusterTransform.SetAnglesAndTranslation(Eigen::Vector3d(0.00000, 0.52400, 0.000000), Eigen::Vector3d(0, 0, 0));
   _headToL515Transform.SetAnglesAndTranslation(Eigen::Vector3d(0.010000, 1.151900, 0.045000),
                                                Eigen::Vector3d(0.275000, 0.052000, 0.140000) - Eigen::Vector3d(0.265000, -0.0200, 0.720000));

   _transformZUp.RotateZ(-90.0f / 180.0f * M_PI);
   _transformZUp.RotateY(90.0f / 180.0f * M_PI);
   _transformZUp.MultiplyLeft(_headToL515Transform);
}



uint8_t PlanarRegionCalculator::CreateParameterBuffer(const ApplicationState& app)
{
   if (app.REGION_MODE == 0)
   {
      float params[] = {(float) app.FILTER_DISPARITY_THRESHOLD, app.MERGE_ANGULAR_THRESHOLD, app.MERGE_DISTANCE_THRESHOLD, (float) app.DEPTH_PATCH_HEIGHT,
                        (float) app.DEPTH_PATCH_WIDTH, (float) app.SUB_H, (float) app.SUB_W, app.DEPTH_FX, app.DEPTH_FY, app.DEPTH_CX, app.DEPTH_CY,
                        (float) app.FILTER_KERNEL_SIZE, (float) app.FILTER_SUB_H, (float) app.FILTER_SUB_W, (float) app.DEPTH_INPUT_HEIGHT,
                        (float) app.DEPTH_INPUT_WIDTH};
      // MS_INFO("Depth ParameterBuffer:({}, {}, {}, {}, {}, {}) Filter:({},{}):{}", app.DEPTH_INPUT_HEIGHT, app.DEPTH_INPUT_WIDTH, app.DEPTH_PATCH_HEIGHT,
      //         app.DEPTH_PATCH_WIDTH, app.SUB_H, app.SUB_W, app.FILTER_SUB_H, app.FILTER_SUB_W, app.FILTER_KERNEL_SIZE);
      return _openCL->CreateLoadBufferFloat(params, sizeof(params) / sizeof(float));
   } else
   {
      float params[] = {(float) app.FILTER_DISPARITY_THRESHOLD, app.HASH_MERGE_ANGULAR_THRESHOLD, app.HASH_MERGE_DISTANCE_THRESHOLD,
                        (float) app.HASH_PATCH_HEIGHT, (float) app.HASH_PATCH_WIDTH, (float) app.HASH_SUB_H, (float) app.HASH_SUB_W, app.DEPTH_FX, app.DEPTH_FY,
                        app.DEPTH_CX, app.DEPTH_CY, (float) app.FILTER_KERNEL_SIZE, (float) app.FILTER_SUB_H, (float) app.FILTER_SUB_W,
                        (float) app.HASH_INPUT_HEIGHT, (float) app.HASH_INPUT_WIDTH, (float) app.HASH_ADJ_SKIPS};
      // MS_INFO("Hash ParameterBuffer:({}, {}, {}, {}, {}, {})", app.HASH_INPUT_HEIGHT, app.HASH_INPUT_WIDTH, app.HASH_PATCH_HEIGHT, app.HASH_PATCH_WIDTH,
      //         app.HASH_SUB_H, app.HASH_SUB_W);
      return _openCL->CreateLoadBufferFloat(params, sizeof(params) / sizeof(float));
   }
}

/*
 * The following are the intrinsic parameters of the depth camera as recorded from L515 RealSense
    height: 480
    width: 640
    distortion_model: "plumb_bob"
    D: [0.0, 0.0, 0.0, 0.0, 0.0]
    K: [459.97265625, 0.0, 341.83984375, 0.0, 459.8046875, 249.173828125, 0.0, 0.0, 1.0]
    R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P: [459.97265625, 0.0, 341.83984375, 0.0, 0.0, 459.8046875, 249.173828125, 0.0, 0.0, 0.0, 1.0, 0.0]
 * */
bool PlanarRegionCalculator::GeneratePatchGraphFromDepth(ApplicationState& appState)
{
   // MS_INFO("Generating Patch Graph on GPU: Color:[{},{}] Depth:[{},{}] Output:[{},{}]", inputColor.cols, inputColor.rows, inputDepth.cols, inputDepth.rows,
   //         output.getRegionOutput().cols, output.getRegionOutput().rows);

   if (inputDepth.rows <= 0 || inputDepth.cols <= 0 || inputDepth.dims <= 0)
   {
      // MS_INFO("Depth image width, height, or dimensions are zero! Skipping Frame.");
      return false;
   }

   appState.REGION_MODE = 0;
   this->app = appState;
   uint8_t paramsBuffer = CreateParameterBuffer(appState);

   cv::Mat depthMat, colorMat;
   {
      // MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::CloneAndBlur");
      depthMat = inputDepth.clone();
      colorMat = inputColor.clone();

      if (appState.EARLY_GAUSSIAN_BLUR)
         GaussianBlur(depthMat, depthMat, cv::Size(appState.GAUSSIAN_SIZE * 2 + 1, appState.GAUSSIAN_SIZE * 2 + 1), appState.GAUSSIAN_SIGMA);
   }

   /* Input Data OpenCL Buffers */
   uint16_t *depthBuffer = reinterpret_cast<uint16_t *>(depthMat.data);
   uint8_t clDepth = _openCL->CreateLoadReadOnlyImage2D_R16(depthBuffer, appState.DEPTH_INPUT_WIDTH, appState.DEPTH_INPUT_HEIGHT);
   //   uint8_t *colorBuffer = reinterpret_cast<uint8_t *>(colorMat.data);
   //   uint8_t clColor = _openCL->CreateLoadReadOnlyImage2D_RGBA8(colorBuffer, appState.DEPTH_INPUT_WIDTH, appState.DEPTH_INPUT_HEIGHT);

   /* Output Data OpenCL Buffers */
   uint8_t clFilterDepth = _openCL->CreateReadWriteImage2D_R16(appState.DEPTH_INPUT_WIDTH, appState.DEPTH_INPUT_HEIGHT);
   uint8_t clBuffer_nx = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_ny = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_nz = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gx = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gy = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gz = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_graph = _openCL->CreateReadWriteImage2D_R8(appState.SUB_W, appState.SUB_H);

   // MS_INFO("Created All Input Images.");

   /* Setting Kernel arguments for patch-packing kernel */
   {
      // MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::SetArgument(s)");

      uint8_t clDepthBuffer = appState.FILTER_SELECTED ? clFilterDepth : clDepth;

      std::vector<uint8_t> argsImgFilter = {clDepth, clFilterDepth, clBuffer_nx};
      for (uint8_t i = 0; i < argsImgFilter.size(); i++)
         _openCL->SetArgument("filterKernel", i, argsImgFilter[i], true);
      _openCL->SetArgument("filterKernel", argsImgFilter.size(), paramsBuffer);

      std::vector<uint8_t> argsImgPack = {clDepthBuffer, clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz};
      for (uint8_t i = 0; i < argsImgPack.size(); i++)
         _openCL->SetArgument("packKernel", i, argsImgPack[i], true);
      _openCL->SetArgument("packKernel", argsImgPack.size(), paramsBuffer);
      _openCL->SetArgument("packKernel", argsImgPack.size() + 1, paramsBuffer);
      _openCL->SetArgumentInt("packKernel", argsImgPack.size() + 2, appState.REGION_MODE);

      std::vector<uint8_t> argsImgMerge = {clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz, clBuffer_graph};
      for (uint8_t i = 0; i < argsImgMerge.size(); i++)
         _openCL->SetArgument("mergeKernel", i, argsImgMerge[i], true);
      _openCL->SetArgument("mergeKernel", argsImgMerge.size(), paramsBuffer);
   }

   /* Setup size for reading patch-wise kernel maps from GPU */
   std::array<cl::size_type, 3> regionOutputSize({appState.SUB_W, appState.SUB_H, 1});
   // regionOutputSize[0] = appState.SUB_W;
   // regionOutputSize[1] = appState.SUB_H;
   // regionOutputSize[2] = 1;
   // cl::size_type origin, size;
   // origin[0] = 0;
   // origin[0] = 0;
   // origin[0] = 0;
   // size[0] = appState.DEPTH_INPUT_WIDTH;
   // size[1] = appState.DEPTH_INPUT_HEIGHT;
   // size[2] = 1;

   std::array<cl::size_type, 3> size({appState.DEPTH_INPUT_WIDTH, appState.DEPTH_INPUT_HEIGHT, 1});

   /* Output Data Buffers on CPU */
   cv::Mat debug(appState.DEPTH_INPUT_HEIGHT, appState.DEPTH_INPUT_WIDTH, CV_16UC1);
   cv::Mat output_nx(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_ny(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_nz(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_gx(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_gy(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_gz(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_graph(appState.SUB_H, appState.SUB_W, CV_8UC1);
   this->filteredDepth = cv::Mat(appState.DEPTH_INPUT_HEIGHT, appState.DEPTH_INPUT_WIDTH, CV_16UC1);

   /* Deploy the patch-packing and patch-merging kernels patch-wise */
   {
      // MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCL::EnqueueNDRangeKernel");
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->filterKernel, cl::NullRange, cl::NDRange(appState.FILTER_SUB_H, appState.FILTER_SUB_W),
                                                 cl::NullRange);
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->packKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->mergeKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
   }
   // MS_INFO("Reading Images Now: ({}, {}, {}, {}, {}, {}, {}, {})", clFilterDepth, clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz,
   //         clBuffer_graph);

   /* Read the output data from OpenCL buffers into CPU buffers */
   {
      // MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCL::ReadImage(s)");
      // _openCL->commandQueue.enqueueReadImage(clDebug, CL_TRUE, origin, size, 0, 0, debug.data);
      _openCL->ReadImage(clFilterDepth, size, filteredDepth.data);
      _openCL->ReadImage(clBuffer_nx, regionOutputSize, output_nx.data);
      _openCL->ReadImage(clBuffer_ny, regionOutputSize, output_ny.data);
      _openCL->ReadImage(clBuffer_nz, regionOutputSize, output_nz.data);
      _openCL->ReadImage(clBuffer_gx, regionOutputSize, output_gx.data);
      _openCL->ReadImage(clBuffer_gy, regionOutputSize, output_gy.data);
      _openCL->ReadImage(clBuffer_gz, regionOutputSize, output_gz.data);
      _openCL->ReadImage(clBuffer_graph, regionOutputSize, output_graph.data);
   }
   /* Synchronize OpenCL to CPU. Block CPU until the entire OpenCL command queue has completed. */
   _openCL->commandQueue.finish();

   /* Combine the CPU buffers into single image with multiple channels */
   cv::Mat regionOutput(appState.SUB_H, appState.SUB_W, CV_32FC(6));
   {
      // MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::Merge");
      std::vector<cv::Mat> channels = {output_nx, output_ny, output_nz, output_gx, output_gy, output_gz};
      merge(channels, regionOutput);
      output.setRegionOutput(regionOutput);
      output.setPatchData(output_graph);
   }

   _openCL->Reset();

   // MS_INFO("Patch Graph Generated on GPU: ({},{},{})", regionOutput.rows, regionOutput.cols, regionOutput.channels());

   return true;
}

void PlanarRegionCalculator::GenerateRegionsFromPointCloud(ApplicationState& appState, std::vector<float>& points)
{
   
   printf("GenerateRegions:(%d, %d, %d, %d, %d, %d)\n", app.HASH_INPUT_HEIGHT, app.HASH_INPUT_WIDTH, app.HASH_PATCH_HEIGHT, app.HASH_PATCH_WIDTH, app.HASH_SUB_H,
           app.HASH_SUB_W);

   printf("Number of Points: %d\n", points.size()/3);

   appState.REGION_MODE = 1;
   _hashMapFrameProcessor->Init(appState);

   /* Setup size for reading patch-wise kernel maps from GPU */
   std::array<cl::size_type, 3> regionOutputSize({appState.HASH_SUB_W, appState.HASH_SUB_H, 1});
   std::array<cl::size_type, 3> size({appState.HASH_INPUT_WIDTH, appState.HASH_INPUT_HEIGHT, 1});

   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    GPU Buffers -----------------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */

   /* Create input data GPU OpenCL Buffers and load vertices into them as float sequence */
   printf("/* Create input data GPU OpenCL Buffers and load vertices into them as float sequence */\n");
   uint8_t paramsBuffer = CreateParameterBuffer(appState);
   uint8_t pointsBuffer = _openCL->CreateLoadBufferFloat(points.data(), points.size());

   /* Create empty intermediate buffers for GPU kernels */
   printf("/* Create empty intermediate buffers for GPU kernels */\n");
   uint8_t indexBuffer = _openCL->CreateBufferInt((int) (points.size() / 3) * 2);
   uint8_t partsBuffer = _openCL->CreateBufferInt((int) (points.size() / 3));
   uint8_t hashBuffer = _openCL->CreateReadWriteImage2D_R16(appState.HASH_INPUT_WIDTH, appState.HASH_INPUT_HEIGHT);

   /*Output Data GPU OpenCL Buffers */
   printf("/*Output Data GPU OpenCL Buffers */\n");
   uint8_t clBuffer_nx = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_ny = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_nz = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_gx = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_gy = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_gz = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_graph = _openCL->CreateReadWriteImage2D_R8(appState.HASH_SUB_W, appState.HASH_SUB_H);

   /* Set arguments for the various GPU kernels to be used */
   printf("/* Set arguments for the various GPU kernels to be used */\n");
   _openCL->SetArgument("indexKernel", 0, pointsBuffer, false);
   _openCL->SetArgument("indexKernel", 1, indexBuffer, false);
   _openCL->SetArgument("indexKernel", 2, partsBuffer, false);
   _openCL->SetArgument("indexKernel", 3, paramsBuffer, false);
   _openCL->SetArgumentInt("indexKernel", 4, points.size());

   // _openCL->SetArgument("hashKernel", 0, pointsBuffer, false);
   // _openCL->SetArgument("hashKernel", 1, indexBuffer, false);
   // _openCL->SetArgument("hashKernel", 2, hashBuffer, true);
   // _openCL->SetArgument("hashKernel", 3, paramsBuffer, false);
   // _openCL->SetArgumentInt("hashKernel", 4, points.size());

   // _openCL->SetArgument("diffuseKernel", 0, hashBuffer, true);
   // _openCL->SetArgument("diffuseKernel", 1, paramsBuffer, false);

   // printf("Setup PackKernel arguments\n");
   // std::vector<uint8_t> argsImgPack = {hashBuffer, clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz};
   // for (uint8_t i = 0; i < argsImgPack.size(); i++)
   //    _openCL->SetArgument("packKernel", i, argsImgPack[i], true);
   // _openCL->SetArgument("packKernel", argsImgPack.size(), paramsBuffer);
   // _openCL->SetArgument("packKernel", argsImgPack.size() + 1, pointsBuffer);
   // _openCL->SetArgumentInt("packKernel", argsImgPack.size() + 2, appState.REGION_MODE);

   // printf("Setup MergeKernel arguments\n");
   // std::vector<uint8_t> argsImgMerge = {clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz, clBuffer_graph};
   // for (uint8_t i = 0; i < argsImgMerge.size(); i++)
   //    _openCL->SetArgument("mergeKernel", i, argsImgMerge[i], true);
   // _openCL->SetArgument("mergeKernel", argsImgMerge.size(), paramsBuffer);

   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    CPU Buffers -----------------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */
   /* Input Data CPU OpenCV Buffers */
   printf("Setup input data CPU buffers\n");
   cv::Mat indexMat(appState.HASH_INPUT_HEIGHT, appState.HASH_INPUT_WIDTH, CV_16UC1, cv::Scalar(0));

   /* Intermediate GPU OpenCV Buffers */
   // printf("Setup GPU buffers\n");
   // cv::Mat output_nx(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   // cv::Mat output_ny(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   // cv::Mat output_nz(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   // cv::Mat output_gx(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   // cv::Mat output_gy(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   // cv::Mat output_gz(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   // cv::Mat output_graph(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_8UC1);


   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    OpenCL Kernel Calls ---------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */
   printf("Deploy OpenCL kernels\n");
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->indexKernel, cl::NullRange, cl::NDRange(points.size() / 3), cl::NullRange);
   // _openCL->commandQueue.enqueueNDRangeKernel(_openCL->hashKernel, cl::NullRange, cl::NDRange(appState.HASH_INPUT_HEIGHT, appState.HASH_INPUT_WIDTH),
   //                                            cl::NullRange);
   // if (appState.HASH_DIFFUSION_ENABLED)
   // {
   //    _openCL->commandQueue.enqueueNDRangeKernel(_openCL->diffuseKernel, cl::NullRange, cl::NDRange(appState.HASH_SUB_H, appState.HASH_SUB_W), cl::NullRange);
   // }

   // _openCL->commandQueue.enqueueNDRangeKernel(_openCL->packKernel, cl::NullRange, cl::NDRange(appState.HASH_SUB_H, appState.HASH_SUB_W), cl::NullRange);
   // _openCL->commandQueue.enqueueNDRangeKernel(_openCL->mergeKernel, cl::NullRange, cl::NDRange(appState.HASH_SUB_H, appState.HASH_SUB_W), cl::NullRange);


   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    OpenCL Read Buffers ---------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */

   printf("Read data from GPU buffers\n");
   // _openCL->ReadImage(clBuffer_nx, regionOutputSize, output_nx.data);
   // _openCL->ReadImage(clBuffer_ny, regionOutputSize, output_ny.data);
   // _openCL->ReadImage(clBuffer_nz, regionOutputSize, output_nz.data);
   // _openCL->ReadImage(clBuffer_gx, regionOutputSize, output_gx.data);
   // _openCL->ReadImage(clBuffer_gy, regionOutputSize, output_gy.data);
   // _openCL->ReadImage(clBuffer_gz, regionOutputSize, output_gz.data);
   // _openCL->ReadImage(clBuffer_graph, regionOutputSize, output_graph.data);

   _openCL->commandQueue.finish();

   // if (appState.HASH_PRINT_MAT)
   // {
   //    _openCL->ReadImage(hashBuffer, size, indexMat.data);
   //    BufferTools::PrintMatR16(indexMat);
   // }

   // if (appState.HASH_PARTS_ENABLED)
   // {
   //    std::vector<int> partIds((int) (points.size() / 3), 0);
   //    _openCL->ReadBufferInt(partsBuffer, partIds.data(), (int) (points.size() / 3));
   //    // cloud.SetPartIds(partIds);
   // } else
   // {
   //    std::vector<int> partIds;
   //    // cloud.SetPartIds(partIds);
   // }

   // printf("Combine outputs from CPU buffers\n");
   // /* Combine the CPU buffers into single image with multiple channels */
   // cv::Mat regionOutput(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC(6));
   // std::vector<cv::Mat> channels = {output_nx, output_ny, output_nz, output_gx, output_gy, output_gz};
   // merge(channels, regionOutput);
   // output.setRegionOutput(regionOutput);
   // output.setPatchData(output_graph);

   // printf("Perform segmentation\n");
   // _hashMapFrameProcessor->GenerateSegmentation(output, planarRegionList); // Perform segmentation using DFS on Patch Graph on CPU to generate Planar Regions
   // PlanarRegion::SetZeroId(planarRegionList);

   // channelMap["HashNormalX"] = output_nx;
   // channelMap["HashNormalY"] = output_ny;
   // channelMap["HashNormalZ"] = output_nz;
   // channelMap["HashCentroidX"] = output_gx;
   // channelMap["HashCentroidY"] = output_gy;
   // channelMap["HashCentroidZ"] = output_gz;

   // _openCL->Reset();

   // printf("Collect planar regions after segmentation.\n");
   // for (int k = 0; k < planarRegionList.size(); k++)
   //    planarRegionList[k]->RetainConvexHull();

   // if (planarRegionList.size() > 0 && appState.EXPORT_REGIONS)
   // {
   //    GeomTools::SaveRegions(planarRegionList, "/Extras/Regions/" +
   //                                             std::string(4 - std::to_string(frameId).length(), '0').append(std::to_string(frameId)) + ".txt");
   //    frameId++;
   // }


   // printf("Total Regions Found: %d\n", planarRegionList.size());
}

void PlanarRegionCalculator::generateRegionsFromDepth(ApplicationState& appState, cv::Mat& depth, double inputTimestamp)
{
   // MS_INFO("Generating Regions");
   appState.REGION_MODE = 0;
   _depthMapFrameProcessor->Init(appState);

   // MS_INFO("Setting Depth");
   inputDepth = depth;
   inputTimestamp = inputTimestamp;

   // MS_INFO("Generating Patch Graph.");

   // Generate patch graph of connected patches on GPU
   bool patchGraphGenerated = GeneratePatchGraphFromDepth(appState);
   if (!patchGraphGenerated)
      return;

   _depthMapFrameProcessor->GenerateSegmentation(output, planarRegionList); // Perform segmentation using DFS on Patch Graph on CPU to generate Planar Regions
   PlanarRegion::SetZeroId(planarRegionList);

   _depthRegionsZUp.clear();
   for (int i = 0; i < planarRegionList.size(); i++)
   {
      std::shared_ptr<PlanarRegion> planarRegion = std::make_shared<PlanarRegion>(planarRegionList[i]->getId());
      planarRegionList[i]->TransformAndFill(planarRegion, _transformZUp);
      _depthRegionsZUp.emplace_back(std::move(planarRegion));
   }

}

void logPlanarRegions(std::vector<std::shared_ptr<PlanarRegion>> planarRegionList)
{
   for (int i = 0; i < planarRegionList.size(); i++)
   {
      // MS_INFO("ID:({}) Center:(%.2f,%.2f,%.2f) Normal:(%.2f,%.2f,%.2f)", planarRegionList[i]->getId(), planarRegionList[i]->GetCenter().x(),
      //         planarRegionList[i]->GetCenter().y(), planarRegionList[i]->GetCenter().z(), planarRegionList[i]->GetNormal().x(),
      //         planarRegionList[i]->GetNormal().y(), planarRegionList[i]->GetNormal().z());
   }
}


void PlanarRegionCalculator::LoadRegions(std::string path, std::vector<std::string>& fileNames, int index)
{
   planarRegionList.clear();
   GeomTools::LoadRegions(index, planarRegionList, path, fileNames);
}

