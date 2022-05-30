package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegion;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtraction;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.avatar.gpuPlanarRegions.GPURegionRing;
import us.ihmc.gdx.visualizers.GDXHeightMapGraphic;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapParameters;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapUpdater;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.perception.ProjectionTools;

import java.nio.ByteBuffer;

public class GDXGPUPlanarRegionExtractionUI
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private final GPUPlanarRegionExtraction gpuPlanarRegionExtraction = new GPUPlanarRegionExtraction();
   private final SimpleGPUHeightMapUpdater simpleGPUHeightMapUpdater = new SimpleGPUHeightMapUpdater(new SimpleGPUHeightMapParameters());
   private final ImFloat mergeDistanceThreshold = new ImFloat();
   private final ImFloat mergeAngularThreshold = new ImFloat();
   private final ImFloat filterDisparityThreshold = new ImFloat();
   private final ImInt desiredPatchSize = new ImInt();
   private final ImInt patchSize = new ImInt(desiredPatchSize.get());
   private final ImInt deadPixelFilterPatchSize = new ImInt();
   private final ImFloat focalLengthXPixels = new ImFloat();
   private final ImFloat focalLengthYPixels = new ImFloat();
   private final ImFloat principalOffsetXPixels = new ImFloat();
   private final ImFloat principalOffsetYPixels = new ImFloat();
   private final ImBoolean earlyGaussianBlur = new ImBoolean();
   private final ImBoolean useFilteredImage = new ImBoolean();
   private final ImBoolean useSVDNormals = new ImBoolean();
   private final ImInt svdReductionFactor = new ImInt();
   private final ImInt gaussianSize = new ImInt();
   private final ImFloat gaussianSigma = new ImFloat();
   private final ImInt searchDepthLimit = new ImInt();
   private final ImInt regionMinPatches = new ImInt();
   private final ImInt boundaryMinPatches = new ImInt();
   private final ImBoolean drawPatches = new ImBoolean(true);
   private final ImBoolean drawBoundaries = new ImBoolean(true);
   private final ImBoolean render3DPlanarRegions = new ImBoolean(true);
   private final ImBoolean render3DHeightMap = new ImBoolean(false);
   private final ImBoolean render3DBoundaries = new ImBoolean(true);
   private final ImBoolean render3DGrownBoundaries = new ImBoolean(true);
   private final ImFloat regionGrowthFactor = new ImFloat();
   private final ImFloat edgeLengthTresholdSlider = new ImFloat(0.224f);
   private final ImFloat triangulationToleranceSlider = new ImFloat(0.0f);
   private final ImInt maxNumberOfIterationsSlider = new ImInt(5000);
   private final ImBoolean allowSplittingConcaveHullChecked = new ImBoolean(false);
   private final ImBoolean removeAllTrianglesWithTwoBorderEdgesChecked = new ImBoolean(false);
   private final ImFloat concaveHullThresholdSlider = new ImFloat(0.15f);
   private final ImFloat shallowAngleThresholdSlider = new ImFloat((float) Math.toRadians(1.0));
   private final ImFloat peakAngleThresholdSlider = new ImFloat((float) Math.toRadians(170.0));
   private final ImFloat lengthThresholdSlider = new ImFloat(0.05f);
   private final ImFloat depthThresholdSlider = new ImFloat(0.10f);
   private final ImInt minNumberOfNodesSlider = new ImInt(10);
   private final ImBoolean cutNarrowPassageChecked = new ImBoolean(true);
   private ImGuiPlot numberOfPlanarRegionsPlot;
   private ImGuiPlot regionMaxSearchDepthPlot;
   private ImGuiPlot numberOfBoundaryVerticesPlot;
   private ImGuiPlot boundaryMaxSearchDepthPlot;
   private ImGuiPlot svdDurationPlot;
   private ImGuiPlot wholeAlgorithmDurationPlot;
   private ImGuiPlot gpuDurationPlot;
   private ImGuiPlot depthFirstSearchDurationPlot;
   private ImGuiPlot planarRegionsSegmentationDurationPlot;
   private ImGuiPlot gpuHeightMapDurationPlot;
   private final Stopwatch wholeAlgorithmDurationStopwatch = new Stopwatch();
   private final Stopwatch gpuDurationStopwatch = new Stopwatch();
   private final Stopwatch depthFirstSearchDurationStopwatch = new Stopwatch();
   private final Stopwatch planarRegionsSegmentationDurationStopwatch = new Stopwatch();
   private final Stopwatch gpuHeightMapStopwatch = new Stopwatch();
   private ImGuiPanel imguiPanel;
   private GDXCVImagePanel blurredDepthPanel;
   private GDXCVImagePanel filteredDepthPanel;
   private GDXCVImagePanel nxImagePanel;
   private GDXCVImagePanel nyImagePanel;
   private GDXCVImagePanel nzImagePanel;
   private GDXCVImagePanel gxImagePanel;
   private GDXCVImagePanel gyImagePanel;
   private GDXCVImagePanel gzImagePanel;
   private GDXCVImagePanel debugExtractionPanel;
   private final Mat BLACK_OPAQUE_RGBA8888 = new Mat((byte) 0, (byte) 0, (byte) 0, (byte) 255);
   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private GDXPlanarRegionsGraphic planarRegionsGraphic;
   private GDXHeightMapGraphic heightMapGraphic;
   private GDXPointCloudRenderer boundaryPointCloud;

   public void create(int imageWidth, int imageHeight, ByteBuffer sourceDepthByteBufferOfFloats, double fx, double fy, double cx, double cy)
   {
      gpuPlanarRegionExtraction.create(imageWidth, imageHeight, sourceDepthByteBufferOfFloats, fx, fy, cx, cy);
//      simpleGPUHeightMapUpdater.create(imageWidth, imageHeight, sourceDepthByteBufferOfFloats, fx, fy, cx, cy);
      simpleGPUHeightMapUpdater.create(imageWidth, imageHeight, gpuPlanarRegionExtraction.getFilteredDepthImage().getBackingDirectByteBuffer(), fx, fy, cx, cy);

      setImGuiWidgetsFromParameters();

      imguiPanel = new ImGuiPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      blurredDepthPanel = new GDXCVImagePanel("Blurred Depth", imageWidth, imageHeight, ImGuiVideoPanel.FLIP_Y);
      filteredDepthPanel = new GDXCVImagePanel("Filtered Depth", imageWidth, imageHeight, ImGuiVideoPanel.FLIP_Y);
      int patchImageWidth = gpuPlanarRegionExtraction.getPatchImageWidth();
      int patchImageHeight = gpuPlanarRegionExtraction.getPatchImageHeight();
      nxImagePanel = new GDXCVImagePanel("Nx Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      nyImagePanel = new GDXCVImagePanel("Ny Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      nzImagePanel = new GDXCVImagePanel("Nz Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      gxImagePanel = new GDXCVImagePanel("Gx Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      gyImagePanel = new GDXCVImagePanel("Gy Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      gzImagePanel = new GDXCVImagePanel("Gz Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      debugExtractionPanel = new GDXCVImagePanel("Planar Region Extraction Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      imguiPanel.addChild(blurredDepthPanel.getVideoPanel());
      imguiPanel.addChild(filteredDepthPanel.getVideoPanel());
      imguiPanel.addChild(nxImagePanel.getVideoPanel());
      imguiPanel.addChild(nyImagePanel.getVideoPanel());
      imguiPanel.addChild(nzImagePanel.getVideoPanel());
      imguiPanel.addChild(gxImagePanel.getVideoPanel());
      imguiPanel.addChild(gyImagePanel.getVideoPanel());
      imguiPanel.addChild(gzImagePanel.getVideoPanel());
      imguiPanel.addChild(debugExtractionPanel.getVideoPanel());

      numberOfPlanarRegionsPlot = new ImGuiPlot(labels.get("Number of planar regions"), 1000, 300, 50);
      regionMaxSearchDepthPlot = new ImGuiPlot(labels.get("Regions max search depth"), 1000, 300, 50);
      numberOfBoundaryVerticesPlot = new ImGuiPlot(labels.get("Number of boundary vertices"), 1000, 300, 50);
      boundaryMaxSearchDepthPlot = new ImGuiPlot(labels.get("Boundary max search depth"), 1000, 300, 50);
      svdDurationPlot = new ImGuiPlot(labels.get("SVD duration"), 1000, 300, 50);
      wholeAlgorithmDurationPlot = new ImGuiPlot(labels.get("Whole algorithm duration"), 1000, 300, 50);
      gpuDurationPlot = new ImGuiPlot(labels.get("GPU processing duration"), 1000, 300, 50);
      depthFirstSearchDurationPlot = new ImGuiPlot(labels.get("Depth first searching duration"), 1000, 300, 50);
      planarRegionsSegmentationDurationPlot = new ImGuiPlot(labels.get("Planar region segmentation duration"), 1000, 300, 50);
      gpuHeightMapDurationPlot = new ImGuiPlot(labels.get("Gpu height map duration"), 1000, 300, 50);

      planarRegionsGraphic = new GDXPlanarRegionsGraphic();
      heightMapGraphic = new GDXHeightMapGraphic();
      boundaryPointCloud = new GDXPointCloudRenderer();
      boundaryPointCloud.create(2000000);

      heightMapGraphic.setRenderGroundPlane(false);
   }

   public void extractPlanarRegions(ReferenceFrame cameraFrame)
   {
      if (!enabled.get())
         return;

      setParametersFromImGuiWidgets();

      wholeAlgorithmDurationStopwatch.start();


      gpuDurationStopwatch.start();

      gpuPlanarRegionExtraction.extractPlanarRegions(cameraFrame, () ->
      {
         int patchImageWidth = gpuPlanarRegionExtraction.getPatchImageWidth();
         int patchImageHeight = gpuPlanarRegionExtraction.getPatchImageHeight();
         OpenCLManager openCLManager = gpuPlanarRegionExtraction.getOpenCLManager();
         nxImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         nyImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         nzImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         gxImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         gyImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         gzImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         debugExtractionPanel.resize(patchImageWidth, patchImageHeight, openCLManager);
      });

      gpuDurationStopwatch.suspend();

      if (debugExtractionPanel.getVideoPanel().getIsShowing().get() && (drawPatches.get() || drawBoundaries.get()))
         debugExtractionPanel.getBytedecoImage().getBytedecoOpenCVMat().setTo(BLACK_OPAQUE_RGBA8888);

      depthFirstSearchDurationStopwatch.start();
      gpuPlanarRegionExtraction.findRegions(island ->
      {
         if (debugExtractionPanel.getVideoPanel().getIsShowing().get() && drawPatches.get())
         {
            for (Point2D regionIndex : island.planarRegion.getRegionIndices())
            {
               int x = (int) regionIndex.getX();
               int y = (int) regionIndex.getY();
               int r = (island.planarRegionIslandIndex + 1) * 312 % 255;
               int g = (island.planarRegionIslandIndex + 1) * 123 % 255;
               int b = (island.planarRegionIslandIndex + 1) * 231 % 255;
               BytePointer pixel = debugExtractionPanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(y, x);
               pixel.put(0, (byte) r);
               pixel.put(1, (byte) g);
               pixel.put(2, (byte) b);
            }
         }
      });
      gpuPlanarRegionExtraction.findBoundariesAndHoles(regionRing ->
      {
         if (debugExtractionPanel.getVideoPanel().getIsShowing().get() && drawBoundaries.get())
         {
            for (Vector2D boundaryIndex : regionRing.getBoundaryIndices())
            {
               int x = (int) boundaryIndex.getX();
               int y = (int) boundaryIndex.getY();
               int r = (regionRing.getIndex() + 1) * 130 % 255;
               int g = (regionRing.getIndex() + 1) * 227 % 255;
               int b = (regionRing.getIndex() + 1) * 332 % 255;
               BytePointer pixel = debugExtractionPanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(y, x);
               pixel.put(0, (byte) r);
               pixel.put(1, (byte) g);
               pixel.put(2, (byte) b);
            }
         }
      });
      gpuPlanarRegionExtraction.growRegionBoundaries();
      depthFirstSearchDurationStopwatch.suspend();

      planarRegionsSegmentationDurationStopwatch.start();
      gpuPlanarRegionExtraction.computePlanarRegions(cameraFrame);
      planarRegionsSegmentationDurationStopwatch.suspend();


      gpuHeightMapStopwatch.start();
      simpleGPUHeightMapUpdater.computeFromDepthMap(cameraFrame.getTransformToWorldFrame());
      gpuHeightMapStopwatch.suspend();

      wholeAlgorithmDurationStopwatch.suspend();

      render2DPanels();
      renderPlanarRegions();
      renderHeightMap();
      renderBoundaryPoints(cameraFrame);
   }

   private void render2DPanels()
   {
      blurredDepthPanel.drawFloatImage(gpuPlanarRegionExtraction.getBlurredDepthImage().getBytedecoOpenCVMat());
      filteredDepthPanel.drawFloatImage(gpuPlanarRegionExtraction.getFilteredDepthImage().getBytedecoOpenCVMat());
      nxImagePanel.drawFloatImage(gpuPlanarRegionExtraction.getNxImage().getBytedecoOpenCVMat());
      nyImagePanel.drawFloatImage(gpuPlanarRegionExtraction.getNyImage().getBytedecoOpenCVMat());
      nzImagePanel.drawFloatImage(gpuPlanarRegionExtraction.getNzImage().getBytedecoOpenCVMat());
      gxImagePanel.drawFloatImage(gpuPlanarRegionExtraction.getCxImage().getBytedecoOpenCVMat());
      gyImagePanel.drawFloatImage(gpuPlanarRegionExtraction.getCyImage().getBytedecoOpenCVMat());
      gzImagePanel.drawFloatImage(gpuPlanarRegionExtraction.getCzImage().getBytedecoOpenCVMat());
      debugExtractionPanel.draw();
   }

   /** FIXME: This method filled with allocations. */
   private void renderPlanarRegions()
   {
      if (!render3DPlanarRegions.get())
         return;

      planarRegionsGraphic.generateMeshes(gpuPlanarRegionExtraction.getPlanarRegionsList());
      planarRegionsGraphic.update();
   }

   private void renderHeightMap()
   {
      if (!render3DHeightMap.get())
         return;

      heightMapGraphic.generateMeshes(simpleGPUHeightMapUpdater.getHeightMap().buildMessage());
      heightMapGraphic.update();
   }

   private void renderBoundaryPoints(ReferenceFrame cameraFrame)
   {
      if (render3DBoundaries.get() || render3DGrownBoundaries.get())
      {
         boundaryPointCloud.prepareVertexBufferForAddingPoints();
         for (GPUPlanarRegion planarRegion : gpuPlanarRegionExtraction.getGPUPlanarRegions())
         {
            if (render3DBoundaries.get())
            {
               if (!planarRegion.getRegionRings().isEmpty())
               {
                  GPURegionRing firstRing = planarRegion.getRegionRings().get(0);
                  for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
                  {
                     int column = (int) boundaryIndex.getX() * gpuPlanarRegionExtraction.getPatchWidth();
                     int row = (int) boundaryIndex.getY() * gpuPlanarRegionExtraction.getPatchHeight();
                     // Note: We are flipping Y here because the input image has Y+ going down
                     float z = gpuPlanarRegionExtraction.getInputFloatDepthImage().getBytedecoOpenCVMat()
                                                        .ptr(gpuPlanarRegionExtraction.getImageHeight() - row, column).getFloat();
                     tempFramePoint.setIncludingFrame(cameraFrame, column, row, z);
                     ProjectionTools.projectDepthPixelToIHMCZUp3D(tempFramePoint,
                                                                  principalOffsetXPixels.get(),
                                                                  principalOffsetYPixels.get(),
                                                                  focalLengthXPixels.get(),
                                                                  focalLengthYPixels.get());
                     tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
                     boundaryPointCloud.putVertex(tempFramePoint);
                  }
               }
            }

            if (render3DGrownBoundaries.get())
            {
               for (Vector3D boundaryVertex : planarRegion.getBoundaryVertices())
               {
                  tempFramePoint.setIncludingFrame(cameraFrame, boundaryVertex);
                  tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
                  boundaryPointCloud.putVertex(tempFramePoint);
               }
            }
         }

         boundaryPointCloud.updateMeshFastest();
      }
   }

   public void renderImGuiWidgets()
   {
      int imageWidth = gpuPlanarRegionExtraction.getImageWidth();
      int imageHeight = gpuPlanarRegionExtraction.getImageHeight();

      ImGui.text("Input image dimensions: " + imageWidth + " x " + imageHeight);
      ImGui.checkbox(labels.get("Enabled"), enabled);
      wholeAlgorithmDurationPlot.render(wholeAlgorithmDurationStopwatch.totalElapsed());
      gpuDurationPlot.render(gpuDurationStopwatch.totalElapsed());
      depthFirstSearchDurationPlot.render(depthFirstSearchDurationStopwatch.totalElapsed());
      planarRegionsSegmentationDurationPlot.render(planarRegionsSegmentationDurationStopwatch.totalElapsed());
      gpuHeightMapDurationPlot.render(gpuHeightMapStopwatch.totalElapsed());
      numberOfPlanarRegionsPlot.render((float) gpuPlanarRegionExtraction.getGPUPlanarRegions().size());
      regionMaxSearchDepthPlot.render((float) gpuPlanarRegionExtraction.getRegionMaxSearchDepth());
      numberOfBoundaryVerticesPlot.render((float) gpuPlanarRegionExtraction.getNumberOfBoundaryPatchesInWholeImage());
      boundaryMaxSearchDepthPlot.render((float) gpuPlanarRegionExtraction.getBoundaryMaxSearchDepth());

      ImGui.checkbox(labels.get("Early gaussian blur"), earlyGaussianBlur);
      ImGui.sliderInt(labels.get("Gaussian size"), gaussianSize.getData(), 1, 20);
      ImGui.sliderFloat(labels.get("Gaussian sigma"), gaussianSigma.getData(), 0.23f, 10.0f);
      if (ImGui.sliderInt(labels.get("Patch size"), desiredPatchSize.getData(), 2, 20))
      {
         if (desiredPatchSize.get() != patchSize.get()
          && imageWidth % desiredPatchSize.get() == 0
          && imageHeight % desiredPatchSize.get() == 0)
         {
            patchSize.set(desiredPatchSize.get());
            gpuPlanarRegionExtraction.setPatchSizeChanged(true);
         }
      }
      ImGui.sliderInt(labels.get("Dead pixel filter patch size"), deadPixelFilterPatchSize.getData(), 1, 20);
      ImGui.checkbox(labels.get("Use filtered image"), useFilteredImage);
      ImGui.sliderFloat(labels.get("Merge distance threshold"), mergeDistanceThreshold.getData(), 0.0f, 0.1f);
      ImGui.sliderFloat(labels.get("Merge angular threshold"), mergeAngularThreshold.getData(), 0.0f, 1.0f);
      ImGui.sliderInt(labels.get("Search depth limit"), searchDepthLimit.getData(), 1, 50000);
      ImGui.sliderInt(labels.get("Region min patches"), regionMinPatches.getData(), 1, 1000);
      ImGui.sliderInt(labels.get("Boundary min patches"), boundaryMinPatches.getData(), 1, 1000);
      ImGui.inputFloat(labels.get("Filter disparity threshold"), filterDisparityThreshold);
      ImGui.sliderFloat(labels.get("Region growth factor"), regionGrowthFactor.getData(), 0.005f, 0.1f);
      ImGui.checkbox(labels.get("Use SVD normals"), useSVDNormals);
      ImGui.sliderInt(labels.get("SVD reduction factor"), svdReductionFactor.getData(), 1, 100);
      svdDurationPlot.render((float) gpuPlanarRegionExtraction.getMaxSVDSolveTime());
      ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      ImGui.checkbox(labels.get("Draw boundaries"), drawBoundaries);
      ImGui.checkbox(labels.get("Render 3D planar regions"), render3DPlanarRegions);
      ImGui.checkbox(labels.get("Render 3D height map"), render3DHeightMap);
      ImGui.checkbox(labels.get("Render 3D boundaries"), render3DBoundaries);
      ImGui.checkbox(labels.get("Render 3D grown boundaries"), render3DGrownBoundaries);
      ImGui.sliderFloat(labels.get("Focal length X (px)"), focalLengthXPixels.getData(), -1000.0f, 1000.0f);
      ImGui.sliderFloat(labels.get("Focal length Y (px)"), focalLengthYPixels.getData(), -1000.0f, 1000.0f);
      ImGui.sliderFloat(labels.get("Principal offset X (px)"), principalOffsetXPixels.getData(), -imageWidth, imageWidth);
      ImGui.sliderFloat(labels.get("Principal offset Y (px)"), principalOffsetYPixels.getData(), -imageHeight, imageHeight);

      ImGui.sliderFloat("Edge Length Threshold", edgeLengthTresholdSlider.getData(), 0, 0.5f);
      ImGui.sliderFloat("Triangulation Tolerance", triangulationToleranceSlider.getData(), 0, 0.3f);
      ImGui.sliderInt("Max Number of Iterations", maxNumberOfIterationsSlider.getData(), 2000, 6000);
      ImGui.checkbox("Remove Degenerate Triangles", removeAllTrianglesWithTwoBorderEdgesChecked);
      ImGui.checkbox("Split Concave Hull", allowSplittingConcaveHullChecked);

      ImGui.sliderFloat("Concave Hull Threshold", concaveHullThresholdSlider.getData(), 0, 1);
      ImGui.sliderFloat("Shallow Angle Threshold", shallowAngleThresholdSlider.getData(), 0, 2.0f * (float) Math.PI);
      ImGui.sliderFloat("Peak Angle Threshold", peakAngleThresholdSlider.getData(), 0, 2.0f * (float) Math.PI);
      ImGui.sliderFloat("Length Threshold", lengthThresholdSlider.getData(), 0, 1);
      ImGui.sliderFloat("Depth Threshold", depthThresholdSlider.getData(), 0, 1);
      ImGui.sliderInt("Min Number of Nodes", minNumberOfNodesSlider.getData(), 0, 100);
      ImGui.checkbox("Cut Narrow Passage", cutNarrowPassageChecked);

      setParametersFromImGuiWidgets();
   }

   public void setImGuiWidgetsFromParameters()
   {
      GPUPlanarRegionExtractionParameters parameters = gpuPlanarRegionExtraction.getParameters();
      mergeDistanceThreshold.set((float) parameters.getMergeDistanceThreshold());
      mergeAngularThreshold.set((float) parameters.getMergeAngularThreshold());
      filterDisparityThreshold.set((float) parameters.getFilterDisparityThreshold());
      desiredPatchSize.set(parameters.getPatchSize());
      patchSize.set(parameters.getPatchSize());
      deadPixelFilterPatchSize.set(parameters.getDeadPixelFilterPatchSize());
      focalLengthXPixels.set((float) parameters.getFocalLengthXPixels());
      focalLengthYPixels.set((float) parameters.getFocalLengthYPixels());
      principalOffsetXPixels.set((float) parameters.getPrincipalOffsetXPixels());
      principalOffsetYPixels.set((float) parameters.getPrincipalOffsetYPixels());
      earlyGaussianBlur.set(parameters.getEarlyGaussianBlur());
      useFilteredImage.set(parameters.getUseFilteredImage());
      useSVDNormals.set(parameters.getUseSVDNormals());
      svdReductionFactor.set(parameters.getSVDReductionFactor());
      gaussianSize.set(parameters.getGaussianSize());
      gaussianSigma.set((float) parameters.getGaussianSigma());
      searchDepthLimit.set(parameters.getSearchDepthLimit());
      regionMinPatches.set(parameters.getRegionMinPatches());
      boundaryMinPatches.set(parameters.getBoundaryMinPatches());
      regionGrowthFactor.set((float) parameters.getRegionGrowthFactor());

      ConcaveHullFactoryParameters concaveHullFactoryParameters = gpuPlanarRegionExtraction.getConcaveHullFactoryParameters();
      edgeLengthTresholdSlider.set((float) concaveHullFactoryParameters.getEdgeLengthThreshold());
      triangulationToleranceSlider.set((float) concaveHullFactoryParameters.getTriangulationTolerance());
      maxNumberOfIterationsSlider.set(concaveHullFactoryParameters.getMaxNumberOfIterations());
      removeAllTrianglesWithTwoBorderEdgesChecked.set(concaveHullFactoryParameters.doRemoveAllTrianglesWithTwoBorderEdges());
      allowSplittingConcaveHullChecked.set(concaveHullFactoryParameters.isSplittingConcaveHullAllowed());

      PolygonizerParameters polygonizerParameters = gpuPlanarRegionExtraction.getPolygonizerParameters();
      concaveHullThresholdSlider.set((float) polygonizerParameters.getConcaveHullThreshold());
      shallowAngleThresholdSlider.set((float) polygonizerParameters.getShallowAngleThreshold());
      peakAngleThresholdSlider.set((float) polygonizerParameters.getPeakAngleThreshold());
      lengthThresholdSlider.set((float) polygonizerParameters.getLengthThreshold());
      depthThresholdSlider.set((float) polygonizerParameters.getDepthThreshold());
      minNumberOfNodesSlider.set(polygonizerParameters.getMinNumberOfNodes());
      cutNarrowPassageChecked.set(polygonizerParameters.getCutNarrowPassage());
   }

   private void setParametersFromImGuiWidgets()
   {
      GPUPlanarRegionExtractionParameters parameters = gpuPlanarRegionExtraction.getParameters();
      parameters.setMergeDistanceThreshold(mergeDistanceThreshold.get());
      parameters.setMergeAngularThreshold(mergeAngularThreshold.get());
      parameters.setFilterDisparityThreshold(filterDisparityThreshold.get());
      parameters.setPatchSize(patchSize.get());
      parameters.setDeadPixelFilterPatchSize(deadPixelFilterPatchSize.get());
      parameters.setFocalLengthXPixels(focalLengthXPixels.get());
      parameters.setFocalLengthYPixels(focalLengthYPixels.get());
      parameters.setPrincipalOffsetXPixels(principalOffsetXPixels.get());
      parameters.setPrincipalOffsetYPixels(principalOffsetYPixels.get());
      parameters.setEarlyGaussianBlur(earlyGaussianBlur.get());
      parameters.setUseFilteredImage(useFilteredImage.get());
      parameters.setUseSVDNormals(useSVDNormals.get());
      parameters.setSVDReductionFactor(svdReductionFactor.get());
      parameters.setGaussianSize(gaussianSize.get());
      parameters.setGaussianSigma(gaussianSigma.get());
      parameters.setSearchDepthLimit(searchDepthLimit.get());
      parameters.setRegionMinPatches(regionMinPatches.get());
      parameters.setBoundaryMinPatches(boundaryMinPatches.get());
      parameters.setRegionGrowthFactor(regionGrowthFactor.get());

      ConcaveHullFactoryParameters concaveHullFactoryParameters = gpuPlanarRegionExtraction.getConcaveHullFactoryParameters();
      concaveHullFactoryParameters.setEdgeLengthThreshold(edgeLengthTresholdSlider.get());
      concaveHullFactoryParameters.setTriangulationTolerance(triangulationToleranceSlider.get());
      concaveHullFactoryParameters.setMaxNumberOfIterations(maxNumberOfIterationsSlider.get());
      concaveHullFactoryParameters.setRemoveAllTrianglesWithTwoBorderEdges(removeAllTrianglesWithTwoBorderEdgesChecked.get());
      concaveHullFactoryParameters.setAllowSplittingConcaveHull(allowSplittingConcaveHullChecked.get());

      PolygonizerParameters polygonizerParameters = gpuPlanarRegionExtraction.getPolygonizerParameters();
      polygonizerParameters.setConcaveHullThreshold(concaveHullThresholdSlider.get());
      polygonizerParameters.setShallowAngleThreshold(shallowAngleThresholdSlider.get());
      polygonizerParameters.setPeakAngleThreshold(peakAngleThresholdSlider.get());
      polygonizerParameters.setLengthThreshold(lengthThresholdSlider.get());
      polygonizerParameters.setDepthThreshold(depthThresholdSlider.get());
      polygonizerParameters.setMinNumberOfNodes(minNumberOfNodesSlider.get());
      polygonizerParameters.setCutNarrowPassage(cutNarrowPassageChecked.get());
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (render3DPlanarRegions.get())
         planarRegionsGraphic.getRenderables(renderables, pool);
      if (render3DGrownBoundaries.get() || render3DBoundaries.get())
         boundaryPointCloud.getRenderables(renderables, pool);
      if (render3DHeightMap.get())
         heightMapGraphic.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      gpuPlanarRegionExtraction.destroy();
      simpleGPUHeightMapUpdater.destroy();
      // TODO: Destroy the rest
   }

   public ImGuiPanel getPanel()
   {
      return imguiPanel;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return gpuPlanarRegionExtraction.getPlanarRegionsList();
   }

   public ImBoolean getEnabled()
   {
      return enabled;
   }

   public ImBoolean getDrawBoundaries()
   {
      return drawBoundaries;
   }

   public ImBoolean getDrawPatches()
   {
      return drawPatches;
   }

   public ImBoolean getRender3DPlanarRegions()
   {
      return render3DPlanarRegions;
   }

   public ImBoolean getRender3DHeightMap()
   {
      return render3DHeightMap;
   }

   public ImBoolean getRender3DBoundaries()
   {
      return render3DBoundaries;
   }

   public ImBoolean getRender3DGrownBoundaries()
   {
      return render3DGrownBoundaries;
   }
}
