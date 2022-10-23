#include "planar_region_calculator.h"

PlanarRegionCalculator::PlanarRegionCalculator(int argc, char **argv, ApplicationState& app) : app(app)
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

// void PlanarRegionCalculator::ImGuiUpdate(ApplicationState& appState)
// {
//    if (ImGui::BeginTabItem("Regions"))
//    {
//       /* Display 2D */
//       ImGui::Text("Input:%d,%d Patch:%d,%d Level:%d", appState.DEPTH_INPUT_HEIGHT, appState.DEPTH_INPUT_WIDTH, appState.DEPTH_PATCH_HEIGHT,
//                   appState.DEPTH_PATCH_WIDTH, appState.KERNEL_SLIDER_LEVEL);
//       ImGui::Checkbox("Generate Regions", &appState.GENERATE_REGIONS);
//       ImGui::Checkbox("Filter", &appState.FILTER_SELECTED);
//       ImGui::Checkbox("Early Gaussian", &appState.EARLY_GAUSSIAN_BLUR);
//       ImGui::Separator();
//       ImGui::Checkbox("Export Regions", &appState.EXPORT_REGIONS);
//       ImGui::Checkbox("Boundary", &appState.SHOW_BOUNDARIES);
//       ImGui::Checkbox("Visual Debug", &appState.VISUAL_DEPTH_DEBUG);
//       ImGui::Checkbox("Show Edges", &appState.SHOW_REGION_EDGES);
//       ImGui::Checkbox("Render 3D", &_render);
//       ImGui::Separator();
//       ImGui::SliderInt("Visual Debug Delay", &appState.VISUAL_DEBUG_DELAY, 1, 100);
//       ImGui::SliderInt("Skip Edges", &appState.NUM_SKIP_EDGES, 1, 20);
//       ImGui::SliderFloat("Display Window Size", &appState.DISPLAY_WINDOW_SIZE, 0.1, 5.0);
//       ImGui::SliderInt("Gaussian Size", &appState.GAUSSIAN_SIZE, 1, 8);
//       ImGui::SliderInt("Gaussian Sigma", &appState.GAUSSIAN_SIGMA, 1, 20);

//       if (ImGui::BeginTabBar("Mode"))
//       {
//          if (ImGui::BeginTabItem("Depth"))
//          {
//             ImGui::Checkbox("Components", &appState.SHOW_DEPTH_REGION_COMPONENTS);
//             ImGui::SliderFloat("Distance Threshold", &appState.MERGE_DISTANCE_THRESHOLD, 0.01f, 1.0f);
//             ImGui::SliderFloat("Angular Threshold", &appState.MERGE_ANGULAR_THRESHOLD, 0.01f, 1.0f);
//             ImGui::SliderFloat("Depth Brightness", &appState.DEPTH_BRIGHTNESS, 1.0, 100.0);

//             ImGui::SliderFloat("X-Angle", &xAngle, -2.0f, 2.0f);
//             ImGui::SliderFloat("Y-Angle", &yAngle, -2.0f, 2.0f);
//             ImGui::SliderFloat("Z-Angle", &zAngle, -2.0f, 2.0f);

//             ImGui::SliderFloat("X-Offset", &xOffset, -2.0f, 2.0f);
//             ImGui::SliderFloat("Y-Offset", &yOffset, -2.0f, 2.0f);
//             ImGui::SliderFloat("Z-Offset", &zOffset, -2.0f, 2.0f);

//             _headToL515Transform.SetAnglesAndTranslation(Eigen::Vector3d(xAngle, yAngle, zAngle), Eigen::Vector3d(xOffset, yOffset, zOffset));

//             _transformZUp.SetToIdentity();
//             _transformZUp.RotateZ(-90.0f / 180.0f * M_PI);
//             _transformZUp.RotateY(90.0f / 180.0f * M_PI);
//             _transformZUp.MultiplyLeft(_headToL515Transform);

//             ImGui::EndTabItem();
//          }

//          if (ImGui::BeginTabItem("Hash"))
//          {
//             ImGui::Checkbox("Diffusion Enabled", &appState.HASH_DIFFUSION_ENABLED);
//             ImGui::SliderInt("Adjacency Skips", &appState.HASH_ADJ_SKIPS, 1, 10);
//             ImGui::SliderFloat("Distance Threshold", &appState.HASH_MERGE_DISTANCE_THRESHOLD, 0.001f, 0.5f);
//             ImGui::SliderFloat("Angular Threshold", &appState.HASH_MERGE_ANGULAR_THRESHOLD, 0.001f, 1.0f);
//             ImGui::Separator();
//             ImGui::Checkbox("Components", &appState.SHOW_HASH_REGION_COMPONENTS);
//             ImGui::Checkbox("Show Hash NX", &appState.SHOW_HASH_NX);
//             ImGui::Checkbox("Show Hash NY", &appState.SHOW_HASH_NY);
//             ImGui::Checkbox("Show Hash NZ", &appState.SHOW_HASH_NZ);
//             ImGui::Checkbox("Show Hash GX", &appState.SHOW_HASH_GX);
//             ImGui::Checkbox("Show Hash GY", &appState.SHOW_HASH_GY);
//             ImGui::Checkbox("Show Hash GZ", &appState.SHOW_HASH_GZ);
//             ImGui::Separator();
//             ImGui::Checkbox("Use Line Mesh", &appState.USE_LINE_MESH);
//             ImGui::Checkbox("Hash Parts Enabled", &appState.HASH_PARTS_ENABLED);
//             ImGui::Checkbox("Print Hash Buffer", &appState.HASH_PRINT_MAT);
//             ImGui::EndTabItem();
//          }

//          ImGui::EndTabBar();
//       }

//       if (ImGui::Button("Hide Display"))
//       {
//          cv::destroyAllWindows();
//       }

//       ImGui::NewLine();
//       ImGuiTools::GetDropDownSelection("File", cloudFiles, depthFileSelected);
//       if (ImGui::Button("Load Cloud"))
//       {
//          //         _regionCalculator->LoadRegions("/home/quantum/Workspace/Volume/catkin_ws/src/MapSenseROS/Extras/Regions/Archive/Set_06_Circle/", fileNames, fileSelected);
//       }
//       if (ImGui::Button("Load Depth"))
//       {
//          //         _regionCalculator->LoadRegions("/home/quantum/Workspace/Volume/catkin_ws/src/MapSenseROS/Extras/Regions/Archive/Set_06_Circle/", fileNames, fileSelected);
//       }
//       ImGui::EndTabItem();
//    }
// }

// void PlanarRegionCalculator::Render()
// {
//    if (this->app.SHOW_DEPTH_REGION_COMPONENTS)
//    {
//       AppUtils::DisplayImage(_depthMapFrameProcessor->GetDebugMat(), app, "Depth Components");
//    }
//    if (this->app.SHOW_HASH_REGION_COMPONENTS)
//    {
//       AppUtils::DisplayImage(_hashMapFrameProcessor->GetDebugMat(), app, "Hash Components");
//    }
//    else
//    {
//       cv::Mat buffer;
//       if (app.SHOW_HASH_NX)
//          buffer = channelMap["HashNormalX"];
//       else if (app.SHOW_HASH_NY)
//          buffer = channelMap["HashNormalY"];
//       else if (app.SHOW_HASH_NZ)
//          buffer = channelMap["HashNormalZ"];
//       else if (app.SHOW_HASH_GX)
//          buffer = channelMap["HashCentroidX"];
//       else if (app.SHOW_HASH_GY)
//          buffer = channelMap["HashCentroidY"];
//       else if (app.SHOW_HASH_GZ)
//          buffer = channelMap["HashCentroidZ"];

//       cv::Mat display;
//       buffer.convertTo(display, CV_8UC1, 255, 0);

//       AppUtils::DisplayImage(display, app, "Hash Components");
//    }
// }

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
   // MAPSENSE_PROFILE_FUNCTION();
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

void PlanarRegionCalculator::GeneratePatchGraphFromPointCloud(ApplicationState& appState, const PointCloud& cloud, double inputTimestamp)
{
   // MAPSENSE_PROFILE_FUNCTION();

   //    int ROWS = 64;
   //    int COLS = 1024;
   //
   //    appState.KERNEL_SLIDER_LEVEL = 2;
   //    appState.DEPTH_INPUT_WIDTH = COLS;
   //    appState.DEPTH_INPUT_HEIGHT = ROWS;
   //    appState.DEPTH_PATCH_HEIGHT = appState.KERNEL_SLIDER_LEVEL;
   //    appState.DEPTH_PATCH_WIDTH = appState.KERNEL_SLIDER_LEVEL;
   //    appState.SUB_H = ROWS / appState.DEPTH_PATCH_HEIGHT;
   //    appState.SUB_W = COLS / appState.DEPTH_PATCH_WIDTH;

   // std::vector<float> points = cloud.GetMesh()->_vertices;
   std::vector<float> points;

   // MS_INFO("GenerateRegions:({}, {}, {}, {}, {}, {})", app.HASH_INPUT_HEIGHT, app.HASH_INPUT_WIDTH, app.HASH_PATCH_HEIGHT, app.HASH_PATCH_WIDTH, app.HASH_SUB_H,
   //         app.HASH_SUB_W);

   appState.REGION_MODE = 1;
   _hashMapFrameProcessor->Init(appState);

   /* Setup size for reading patch-wise kernel maps from GPU */
   std::array<cl::size_type, 3> regionOutputSize({appState.HASH_SUB_W, appState.HASH_SUB_H, 1});
   // regionOutputSize[0] = appState.HASH_SUB_W;
   // regionOutputSize[1] = appState.HASH_SUB_H;
   // regionOutputSize[2] = 1;
   // cl::size_type<3> origin, size;
   // origin[0] = 0;
   // origin[0] = 0;
   // origin[0] = 0;
   // size[0] = appState.HASH_INPUT_WIDTH;
   // size[1] = appState.HASH_INPUT_HEIGHT;
   // size[2] = 1;


   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    GPU Buffers -----------------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */
   /* Input Data GPU OpenCL Buffers */
   uint8_t paramsBuffer = CreateParameterBuffer(appState);
   uint8_t pointsBuffer = _openCL->CreateLoadBufferFloat(points.data(), points.size());

   /* Intermediate GPU OpenCL Buffers */
   uint8_t indexBuffer = _openCL->CreateBufferInt((int) (points.size() / 3) * 2);
   uint8_t partsBuffer = _openCL->CreateBufferInt((int) (points.size() / 3));
   uint8_t hashBuffer = _openCL->CreateReadWriteImage2D_R16(appState.HASH_INPUT_WIDTH, appState.HASH_INPUT_HEIGHT);

   /*Output Data GPU OpenCL Buffers */
   uint8_t clBuffer_nx = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_ny = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_nz = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_gx = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_gy = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_gz = _openCL->CreateReadWriteImage2D_RFloat(appState.HASH_SUB_W, appState.HASH_SUB_H);
   uint8_t clBuffer_graph = _openCL->CreateReadWriteImage2D_R8(appState.HASH_SUB_W, appState.HASH_SUB_H);

   // MS_INFO("Created All Input Images.");

   _openCL->SetArgument("indexKernel", 0, pointsBuffer, false);
   _openCL->SetArgument("indexKernel", 1, indexBuffer, false);
   _openCL->SetArgument("indexKernel", 2, partsBuffer, false);
   _openCL->SetArgument("indexKernel", 3, paramsBuffer, false);
   _openCL->SetArgumentInt("indexKernel", 4, points.size());

   _openCL->SetArgument("hashKernel", 0, pointsBuffer, false);
   _openCL->SetArgument("hashKernel", 1, indexBuffer, false);
   _openCL->SetArgument("hashKernel", 2, hashBuffer, true);
   _openCL->SetArgument("hashKernel", 3, paramsBuffer, false);
   _openCL->SetArgumentInt("hashKernel", 4, points.size());

   _openCL->SetArgument("diffuseKernel", 0, hashBuffer, true);
   _openCL->SetArgument("diffuseKernel", 1, paramsBuffer, false);

   std::vector<uint8_t> argsImgPack = {hashBuffer, clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz};
   for (uint8_t i = 0; i < argsImgPack.size(); i++)
      _openCL->SetArgument("packKernel", i, argsImgPack[i], true);
   _openCL->SetArgument("packKernel", argsImgPack.size(), paramsBuffer);
   _openCL->SetArgument("packKernel", argsImgPack.size() + 1, pointsBuffer);
   _openCL->SetArgumentInt("packKernel", argsImgPack.size() + 2, appState.REGION_MODE);

   std::vector<uint8_t> argsImgMerge = {clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz, clBuffer_graph};
   for (uint8_t i = 0; i < argsImgMerge.size(); i++)
      _openCL->SetArgument("mergeKernel", i, argsImgMerge[i], true);
   _openCL->SetArgument("mergeKernel", argsImgMerge.size(), paramsBuffer);

   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    CPU Buffers -----------------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */
   /* Input Data CPU OpenCV Buffers */
   cv::Mat indexMat(appState.HASH_INPUT_HEIGHT, appState.HASH_INPUT_WIDTH, CV_16UC1, cv::Scalar(0));

   /* Intermediate GPU OpenCV Buffers */
   cv::Mat output_nx(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   cv::Mat output_ny(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   cv::Mat output_nz(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   cv::Mat output_gx(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   cv::Mat output_gy(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   cv::Mat output_gz(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC1);
   cv::Mat output_graph(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_8UC1);


   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    OpenCL Kernel Calls ---------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->indexKernel, cl::NullRange, cl::NDRange(points.size() / 3), cl::NullRange);
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->hashKernel, cl::NullRange, cl::NDRange(appState.HASH_INPUT_HEIGHT, appState.HASH_INPUT_WIDTH),
                                              cl::NullRange);
   if (appState.HASH_DIFFUSION_ENABLED)
   {
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->diffuseKernel, cl::NullRange, cl::NDRange(appState.HASH_SUB_H, appState.HASH_SUB_W), cl::NullRange);
   }

   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->packKernel, cl::NullRange, cl::NDRange(appState.HASH_SUB_H, appState.HASH_SUB_W), cl::NullRange);
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->mergeKernel, cl::NullRange, cl::NDRange(appState.HASH_SUB_H, appState.HASH_SUB_W), cl::NullRange);


   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    OpenCL Read Buffers ---------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */

   auto start_point = std::chrono::steady_clock::now();
   _openCL->ReadImage(clBuffer_nx, regionOutputSize, output_nx.data);
   _openCL->ReadImage(clBuffer_ny, regionOutputSize, output_ny.data);
   _openCL->ReadImage(clBuffer_nz, regionOutputSize, output_nz.data);
   _openCL->ReadImage(clBuffer_gx, regionOutputSize, output_gx.data);
   _openCL->ReadImage(clBuffer_gy, regionOutputSize, output_gy.data);
   _openCL->ReadImage(clBuffer_gz, regionOutputSize, output_gz.data);
   _openCL->ReadImage(clBuffer_graph, regionOutputSize, output_graph.data);

   _openCL->commandQueue.finish();
   auto end_point = std::chrono::steady_clock::now();

   // if (appState.HASH_PRINT_MAT)
   // {
   //    _openCL->ReadImage(hashBuffer, size, indexMat.data);
   //    AppUtils::PrintMatR16(indexMat);
   // }

   if (appState.HASH_PARTS_ENABLED)
   {
      std::vector<int> partIds((int) (points.size() / 3), 0);
      _openCL->ReadBufferInt(partsBuffer, partIds.data(), (int) (points.size() / 3));
      // cloud.SetPartIds(partIds);
   } else
   {
      std::vector<int> partIds;
      // cloud.SetPartIds(partIds);
   }

   /* Combine the CPU buffers into single image with multiple channels */
   cv::Mat regionOutput(appState.HASH_SUB_H, appState.HASH_SUB_W, CV_32FC(6));
   std::vector<cv::Mat> channels = {output_nx, output_ny, output_nz, output_gx, output_gy, output_gz};
   merge(channels, regionOutput);
   output.setRegionOutput(regionOutput);
   output.setPatchData(output_graph);

   _hashMapFrameProcessor->GenerateSegmentation(output, planarRegionList); // Perform segmentation using DFS on Patch Graph on CPU to generate Planar Regions
   PlanarRegion::SetZeroId(planarRegionList);

   channelMap["HashNormalX"] = output_nx;
   channelMap["HashNormalY"] = output_ny;
   channelMap["HashNormalZ"] = output_nz;
   channelMap["HashCentroidX"] = output_gx;
   channelMap["HashCentroidY"] = output_gy;
   channelMap["HashCentroidZ"] = output_gz;

   _openCL->Reset();

   //   _hashRegionsZUp.clear();
   //   for (int i = 0; i < planarRegionList.size(); i++)
   //   {
   //      std::shared_ptr<PlanarRegion> planarRegion = std::make_shared<PlanarRegion>(planarRegionList[i]->getId());
   //      planarRegionList[i]->TransformAndFill(planarRegion, _transformZUp);
   //      _hashRegionsZUp.emplace_back(std::move(planarRegion));
   //   }

   for (int k = 0; k < planarRegionList.size(); k++)
      planarRegionList[k]->RetainConvexHull();

   if (planarRegionList.size() > 0 && appState.EXPORT_REGIONS)
   {
      GeomTools::SaveRegions(planarRegionList, "/Extras/Regions/" +
                                               std::string(4 - std::to_string(frameId).length(), '0').append(std::to_string(frameId)) + ".txt");
      frameId++;
   }

   long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
   long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();

   float duration = (end - start) * 0.001f;

   // MS_INFO("Total Time PR: {} ms", duration);
   // MS_INFO("Total Regions Found: {}", planarRegionList.size());
}

void PlanarRegionCalculator::generateRegionsFromDepth(ApplicationState& appState, cv::Mat& depth, double inputTimestamp)
{
   // MAPSENSE_PROFILE_FUNCTION();
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

   /* Planar Regions Ready To Be Published Right Here. */
   // MS_INFO("Number of Planar Regions: {}", planarRegionList.size());
   if (appState.EXPORT_REGIONS)
   {
      if (frameId % 10 == 0)
      {
         GeomTools::SaveRegions(planarRegionList, "/Extras/Regions/" +
                                                  std::string(4 - std::to_string(frameId).length(), '0').append(std::to_string(frameId)) + ".txt");
      }
      frameId++;
   }
   //   extractRealPlanes();
}

void PlanarRegionCalculator::onMouse(int event, int x, int y, int flags, void *userdata)
{
   MapFrame out = *((MapFrame *) userdata);
   if (event == cv::EVENT_MOUSEMOVE)
   {
      // MS_INFO("[{},{}]:", y / 8, x / 8);
      // MS_INFO("%hu ", out.getPatchData().at<uint8_t>(y / 8, x / 8));
      cv::Vec6f patch = out.getRegionOutput().at<cv::Vec6f>(y / 8, x / 8);
      // MS_INFO("Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf)\n", patch[3], patch[4], patch[5], patch[0], patch[1], patch[2]);
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

bool PlanarRegionCalculator::RenderEnabled()
{
   return _render;
}

void PlanarRegionCalculator::GenerateRegionFromPointcloudOnCPU()
{
   //    float pitchUnit = M_PI / (2 * ROWS);
   //    float yawUnit = 2 * M_PI / (COLS);
   //    for (uint16_t i = 0; i < (uint16_t) (points.size() / 3); i++)
   //    {
   //        float x = points[i * 3];
   //        float y = points[i * 3 + 1];
   //        float z = points[i * 3 + 2];
   //
   //        float radius = sqrt(x * x + y * y);
   //
   //        float pitch = atan2(z, radius);
   //        int pitchCount = 32 + (int) (pitch / pitchUnit);
   //
   //        float yaw = atan2(-y, x);
   //        int yawCount = 512 + (int) (yaw / yawUnit);
   //
   //        if (pitchCount >= 0 && pitchCount < ROWS && yawCount >= 0 && yawCount < COLS)
   //        {
   //            /* countMat.at<char>(pitchCount, yawCount) */
   //            indexMat.at<uint16_t>(pitchCount, yawCount) = i;
   //            countMat.at<char>(pitchCount, yawCount) += 1;
   //            //         printf("r:%d, c:%d, i:%d\t", pitchCount, yawCount, indexMat.at<uint16_t>(pitchCount, yawCount, 0) = i);
   //        }
   ////        printf("\n");
   //        //       printf("X: %.2lf, Y:%.2lf, Z:%.2lf, Pitch:%.2lf, Yaw:%.2lf, pc:%d, yc:%d\n", x, y, z, pitch, yaw, pitchCount, yawCount);
   //    }
   //
   //    /* Patch Packing */
   //    for (int i = 0; i < appState.HASH_SUB_H; i++)
   //    {
   //        for (int j = 0; j < appState.HASH_SUB_W; j++)
   //        {
   //            Eigen::Vector3f normal = Eigen::Vector3f::Zero();
   //            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
   //            Eigen::Vector3f va = Eigen::Vector3f::Zero();
   //            Eigen::Vector3f vb = Eigen::Vector3f::Zero();
   //            Eigen::Vector3f vc = Eigen::Vector3f::Zero();
   //            Eigen::Vector3f vd = Eigen::Vector3f::Zero();
   //            uint16_t indexA = 0, indexB = 0, indexC = 0, indexD = 0;
   //            uint8_t totalCount = 0, normalCount = 0;
   //            for (int m = 0; m < appState.HASH_PATCH_HEIGHT - 1; m++)
   //            {
   //                for (int n = 0; n < appState.HASH_PATCH_WIDTH - 1; n++)
   //                {
   //                    indexA = indexMat.at<uint16_t>(i + m, j + n);
   //                    va = Eigen::Vector3f(points[indexA * 3], points[indexA * 3 + 1], points[indexA * 3 + 2]);
   //
   //                    indexB = indexMat.at<uint16_t>(i + m, j + n + 1);
   //                    vb = Eigen::Vector3f(points[indexB * 3], points[indexB * 3 + 1], points[indexB * 3 + 2]);
   //
   //                    indexC = indexMat.at<uint16_t>(i + m + 1, j + n);
   //                    vc = Eigen::Vector3f(points[indexC * 3], points[indexC * 3 + 1], points[indexC * 3 + 2]);
   //
   //                    indexD = indexMat.at<uint16_t>(i + m + 1, j + n + 1);
   //                    vd = Eigen::Vector3f(points[indexD * 3], points[indexD * 3 + 1], points[indexD * 3 + 2]);
   //
   //                    totalCount = (indexA != 0) + (indexB != 0) + (indexC != 0) + (indexD != 0);
   //
   //                    if (indexA != 0)
   //                        centroid += va;
   //                    if (indexB != 0)
   //                        centroid += vb;
   //                    if (indexC != 0)
   //                        centroid += vc;
   //                    if (indexD != 0)
   //                        centroid += vd;
   //                    if (totalCount > 0)
   //                        centroid = centroid / (float) totalCount;
   //
   //                    if (indexA != 0 && indexB != 0 && indexC != 0)
   //                    {
   //                        Eigen::Vector3f nmlA = ((vb));
   //                        normal += nmlA;
   ////                        printf("A:normal:(%.2lf,%.2lf,%.2lf)\n", nmlA.x(), nmlA.y(), nmlA.z());
   //                        normalCount++;
   //                    }
   //                    if (indexA != 0 && indexB != 0 && indexC != 0)
   //                    {
   //                        Eigen::Vector3f nmlB = ((vc));
   //                        normal += nmlB;
   ////                        printf("B:normal:(%.2lf,%.2lf,%.2lf)\n", nmlB.x(), nmlB.y(), nmlB.z());
   //                        normalCount++;
   //                    }
   //                    if (indexA != 0 && indexB != 0 && indexC != 0)
   //                    {
   //                        Eigen::Vector3f nmlC = ((vd));
   //                        normal += nmlC;
   ////                        printf("C:normal:(%.2lf,%.2lf,%.2lf)\n", nmlC.x(), nmlC.y(), nmlC.z());
   //                        normalCount++;
   //                    }
   //                    if (indexA != 0 && indexB != 0 && indexC != 0)
   //                    {
   //                        Eigen::Vector3f nmlD = ((va));
   //                        normal += nmlD;
   ////                        printf("D:normal:(%.2lf,%.2lf,%.2lf)\n", nmlD.x(), nmlD.y(), nmlD.z());
   //                        normalCount++;
   //                    }
   //
   //                    normal = normal.normalized();
   //
   //                    //               std::cout << va << std::endl << vb << std::endl << vc << std::endl << vd << std::endl;
   //
   ////                    printf(
   ////                            "Total: %d, i:%d, j:%d, ia:%d, ib:%d, ic:%d, id:%d, ncount:%d, cA:%d, cB:%d, cC:%d, cD:%d, normal:(%.2lf,%.2lf,%.2lf), centroid:(%.2lf,%.2lf,%.2lf)\n",
   ////                            (uint16_t) (points.size() / 3), i, j, indexA, indexB, indexC, indexD, normalCount, countA, countB, countC, countD, normal.x(), normal.y(),
   ////                            normal.z(), centroid.x(), centroid.y(), centroid.z());
   //                }
   //            }
   //
   //            output_nx.at<float>(i, j) = normal.x();
   //            output_ny.at<float>(i, j) = normal.y();
   //            output_nz.at<float>(i, j) = normal.z();
   //            output_gx.at<float>(i, j) = centroid.x();
   //            output_gy.at<float>(i, j) = centroid.y();
   //            output_gz.at<float>(i, j) = centroid.z();
   //        }
   //    }

   //    /* Patch Merging */
   //    for (int i = 1; i < appState.HASH_SUB_H - 1; i++)
   //    {
   //        for (int j = 1; j < appState.HASH_SUB_W - 1; j++)
   //        {
   //            uint8_t patch = 0;
   //            int count = 0;
   //
   //            Eigen::Vector3f n_a(output_nx.at<float>(i, j), output_ny.at<float>(i, j), output_nz.at<float>(i, j));
   //            Eigen::Vector3f g_a(output_gx.at<float>(i, j), output_gy.at<float>(i, j), output_gz.at<float>(i, j));
   //
   //            for (int k = -1; k <= 1; k += 1)
   //            {
   //                for (int l = -1; l <= 1; l += 1)
   //                {
   //                    if (!(k == 0 && l == 0))
   //                    {
   //                        Eigen::Vector3f n_b(output_nx.at<float>(i+k, j+l), output_ny.at<float>(i+k, j+l), output_nz.at<float>(i+k, j+l));
   //                        Eigen::Vector3f g_b(output_gx.at<float>(i+k, j+l), output_gy.at<float>(i+k, j+l), output_gz.at<float>(i+k, j+l));
   //
   //                        if(GeomTools::CheckPatchConnection(g_a, n_a, g_b, n_b, 0.1, 0.7))
   //                        {
   //                            patch = (1 << count) | patch;
   //                        }
   //                        count++;
   //                    }
   //                }
   //            }
   //            output_graph.at<uint8_t>(i,j) = patch;
   //        }
   //    }

   //    AppUtils::PrintMatR8(output_graph, 0, false, 0, 0);
}
