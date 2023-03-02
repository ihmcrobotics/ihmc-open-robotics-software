package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.rapidRegions.RapidPlanarRegion;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionIsland;
import us.ihmc.perception.rapidRegions.RapidRegionRing;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtraction;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.visualizers.RDXHeightMapGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapParameters;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapUpdater;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.perception.ProjectionTools;
import us.ihmc.tools.thread.GuidedSwapReference;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.nio.ByteBuffer;

public class RDXGPUPlanarRegionExtractionUI
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private final GPUPlanarRegionExtraction gpuPlanarRegionExtraction = new GPUPlanarRegionExtraction();
   private final SimpleGPUHeightMapUpdater simpleGPUHeightMapUpdater = new SimpleGPUHeightMapUpdater(new SimpleGPUHeightMapParameters());
   private ImGuiStoredPropertySetTuner gpuRegionParametersTuner;
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private final ImInt appliedPatchSize = new ImInt(gpuPlanarRegionExtraction.getParameters().getPatchSize());
   private final ImBoolean drawPatches = new ImBoolean(true);
   private final ImBoolean drawBoundaries = new ImBoolean(true);
   private final ImBoolean render3DPlanarRegions = new ImBoolean(true);
   private final ImBoolean render3DHeightMap = new ImBoolean(false);
   private final ImBoolean render3DBoundaries = new ImBoolean(true);
   private final ImBoolean render3DGrownBoundaries = new ImBoolean(true);
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
   private final YoDouble wholeAlgorithmDuration = new YoDouble("wholeAlgorithmDuration", yoRegistry);
   private final Stopwatch gpuDurationStopwatch = new Stopwatch();
   private final Stopwatch depthFirstSearchDurationStopwatch = new Stopwatch();
   private final Stopwatch planarRegionsSegmentationDurationStopwatch = new Stopwatch();
   private final Stopwatch gpuHeightMapStopwatch = new Stopwatch();
   private ImGuiPanel imguiPanel;
   private RDXBytedecoImagePanel blurredDepthPanel;
   private RDXBytedecoImagePanel filteredDepthPanel;
   private RDXBytedecoImagePanel nxImagePanel;
   private RDXBytedecoImagePanel nyImagePanel;
   private RDXBytedecoImagePanel nzImagePanel;
   private RDXBytedecoImagePanel gxImagePanel;
   private RDXBytedecoImagePanel gyImagePanel;
   private RDXBytedecoImagePanel gzImagePanel;
   private RDXBytedecoImagePanel debugExtractionPanel;
   private final Mat BLACK_OPAQUE_RGBA8888 = new Mat((byte) 0, (byte) 0, (byte) 0, (byte) 255);
   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private RDXPlanarRegionsGraphic planarRegionsGraphic;
   private RDXHeightMapGraphic heightMapGraphic;
   private GuidedSwapReference<RDXPointCloudRenderer> boundaryPointCloudSwap;
   private Runnable pointCloudRendererShutdown;
   private Array<Renderable> latestRenderables;
   private Pool<Renderable> latestPool;
   private ReferenceFrame cameraFrame;

   public void create(int imageWidth,
                      int imageHeight,
                      ByteBuffer sourceDepthByteBufferOfFloats,
                      double fx,
                      double fy,
                      double cx,
                      double cy,
                      ReferenceFrame cameraFrame)
   {
      this.cameraFrame = cameraFrame;
      gpuPlanarRegionExtraction.create(imageWidth, imageHeight, sourceDepthByteBufferOfFloats, fx, fy, cx, cy);
//      simpleGPUHeightMapUpdater.create(imageWidth, imageHeight, sourceDepthByteBufferOfFloats, fx, fy, cx, cy);
      simpleGPUHeightMapUpdater.create(imageWidth, imageHeight, gpuPlanarRegionExtraction.getFilteredDepthImage().getBackingDirectByteBuffer(), fx, fy, cx, cy);

      gpuRegionParametersTuner = new ImGuiStoredPropertySetTuner(gpuPlanarRegionExtraction.getParameters().getTitle());
      gpuRegionParametersTuner.create(gpuPlanarRegionExtraction.getParameters());

      setImGuiWidgetsFromParameters();

      imguiPanel = new ImGuiPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      blurredDepthPanel = new RDXBytedecoImagePanel("Blurred Depth", imageWidth, imageHeight, RDXImagePanel.FLIP_Y);
      filteredDepthPanel = new RDXBytedecoImagePanel("Filtered Depth", imageWidth, imageHeight, RDXImagePanel.FLIP_Y);
      int patchImageWidth = gpuPlanarRegionExtraction.getPatchImageWidth();
      int patchImageHeight = gpuPlanarRegionExtraction.getPatchImageHeight();
      nxImagePanel = new RDXBytedecoImagePanel("Nx Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      nyImagePanel = new RDXBytedecoImagePanel("Ny Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      nzImagePanel = new RDXBytedecoImagePanel("Nz Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      gxImagePanel = new RDXBytedecoImagePanel("Gx Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      gyImagePanel = new RDXBytedecoImagePanel("Gy Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      gzImagePanel = new RDXBytedecoImagePanel("Gz Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      debugExtractionPanel = new RDXBytedecoImagePanel("Planar Region Extraction Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      imguiPanel.addChild(blurredDepthPanel.getImagePanel());
      imguiPanel.addChild(filteredDepthPanel.getImagePanel());
      imguiPanel.addChild(nxImagePanel.getImagePanel());
      imguiPanel.addChild(nyImagePanel.getImagePanel());
      imguiPanel.addChild(nzImagePanel.getImagePanel());
      imguiPanel.addChild(gxImagePanel.getImagePanel());
      imguiPanel.addChild(gyImagePanel.getImagePanel());
      imguiPanel.addChild(gzImagePanel.getImagePanel());
      imguiPanel.addChild(debugExtractionPanel.getImagePanel());

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

      planarRegionsGraphic = new RDXPlanarRegionsGraphic();
      heightMapGraphic = new RDXHeightMapGraphic();
      boundaryPointCloudSwap = new GuidedSwapReference<>(() ->
      {

         RDXPointCloudRenderer boundaryPointCloud = new RDXPointCloudRenderer();
         boundaryPointCloud.create(2000000);
         pointCloudRendererShutdown = boundaryPointCloud::dispose;
         return boundaryPointCloud;
      }, this::boundaryPointCloudUpdateOnLowPriorityThread, this::boundaryPointCloudUpdateOnHighPriorityThread);

      heightMapGraphic.setRenderGroundPlane(false);
   }

   volatile boolean processing = false;
   volatile boolean needToDraw = false;

   public void extractPlanarRegions()
   {
      extractPlanarRegions(null);
   }

   public void extractPlanarRegions(Runnable runWhenFinished)
   {
      if (enabled.get())
      {
         if (!processing)
         {
            processing = true;
            setParametersFromImGuiWidgets();
            wholeAlgorithmDurationStopwatch.start();
            gpuDurationStopwatch.start();
            gpuPlanarRegionExtraction.readFromSourceImage();
            ThreadTools.startAsDaemon(() -> processingThread(runWhenFinished), getClass().getSimpleName() + "Processing");
         }

         if (needToDraw)
         {
            needToDraw = false;

            render2DPanels();
            renderPlanarRegions();
            renderHeightMap();
            renderBoundaryPoints(cameraFrame);

            processing = false;
         }
      }
   }

   private void processingThread(Runnable runWhenFinished)
   {
      gpuPlanarRegionExtraction.extractPlanarRegions(this::onPatchSizeChanged);

      gpuDurationStopwatch.suspend();

      if (debugExtractionPanel.getImagePanel().getIsShowing().get() && (drawPatches.get() || drawBoundaries.get()))
         debugExtractionPanel.getBytedecoImage().getBytedecoOpenCVMat().setTo(BLACK_OPAQUE_RGBA8888);

      depthFirstSearchDurationStopwatch.start();
      gpuPlanarRegionExtraction.findRegions(this::forDrawingRegionsInDebugPanel);
      gpuPlanarRegionExtraction.findBoundariesAndHoles(this::forDrawingRingsInDebugPanel);
      gpuPlanarRegionExtraction.growRegionBoundaries();
      depthFirstSearchDurationStopwatch.suspend();

      planarRegionsSegmentationDurationStopwatch.start();
      gpuPlanarRegionExtraction.computePlanarRegions(cameraFrame);
      planarRegionsSegmentationDurationStopwatch.suspend();

      gpuHeightMapStopwatch.start();
      //      simpleGPUHeightMapUpdater.computeFromDepthMap(cameraFrame.getTransformToWorldFrame());
      gpuHeightMapStopwatch.suspend();

      wholeAlgorithmDurationStopwatch.suspend();
      wholeAlgorithmDuration.set(wholeAlgorithmDurationStopwatch.lapElapsed());

      if (runWhenFinished != null)
         runWhenFinished.run();

      needToDraw = true;
   }

   private void onPatchSizeChanged()
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
   }

   private void forDrawingRegionsInDebugPanel(RapidPlanarRegionIsland island)
   {
      if (debugExtractionPanel.getImagePanel().getIsShowing().get() && drawPatches.get())
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
   }

   private void forDrawingRingsInDebugPanel(RapidRegionRing regionRing)
   {
      if (debugExtractionPanel.getImagePanel().getIsShowing().get() && drawBoundaries.get())
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
   }

   private void render2DPanels()
   {
      blurredDepthPanel.drawDepthImage(gpuPlanarRegionExtraction.getBlurredDepthImage().getBytedecoOpenCVMat());
      filteredDepthPanel.drawDepthImage(gpuPlanarRegionExtraction.getFilteredDepthImage().getBytedecoOpenCVMat());
      nxImagePanel.drawDepthImage(gpuPlanarRegionExtraction.getNxImage().getBytedecoOpenCVMat());
      nyImagePanel.drawDepthImage(gpuPlanarRegionExtraction.getNyImage().getBytedecoOpenCVMat());
      nzImagePanel.drawDepthImage(gpuPlanarRegionExtraction.getNzImage().getBytedecoOpenCVMat());
      gxImagePanel.drawDepthImage(gpuPlanarRegionExtraction.getCxImage().getBytedecoOpenCVMat());
      gyImagePanel.drawDepthImage(gpuPlanarRegionExtraction.getCyImage().getBytedecoOpenCVMat());
      gzImagePanel.drawDepthImage(gpuPlanarRegionExtraction.getCzImage().getBytedecoOpenCVMat());
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

//      heightMapGraphic.generateMeshes(simpleGPUHeightMapUpdater.getHeightMap().buildMessage());
      heightMapGraphic.update();
   }

   private void renderBoundaryPoints(ReferenceFrame cameraFrame)
   {
      if (render3DBoundaries.get() || render3DGrownBoundaries.get())
      {
         boundaryPointCloudSwap.accessOnLowPriorityThread();
      }
   }

   private void boundaryPointCloudUpdateOnLowPriorityThread(RDXPointCloudRenderer boundaryPointCloud)
   {
      boundaryPointCloud.prepareVertexBufferForAddingPoints();
      for (RapidPlanarRegion planarRegion : gpuPlanarRegionExtraction.getRapidPlanarRegions())
      {
         if (render3DBoundaries.get())
         {
            if (!planarRegion.getRegionRings().isEmpty())
            {
               RapidRegionRing firstRing = planarRegion.getRegionRings().get(0);
               for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
               {
                  int column = (int) boundaryIndex.getX() * gpuPlanarRegionExtraction.getPatchWidth();
                  int row = (int) boundaryIndex.getY() * gpuPlanarRegionExtraction.getPatchHeight();
                  // Note: We are flipping Y here because the input image has Y+ going down
                  float z = gpuPlanarRegionExtraction.getInputFloatDepthImage().getBytedecoOpenCVMat()
                                                     .ptr(gpuPlanarRegionExtraction.getImageHeight() - row, column).getFloat();
                  tempFramePoint.setIncludingFrame(cameraFrame, column, row, z);
                  ProjectionTools.projectDepthPixelToIHMCZUp3D(tempFramePoint,
                                                               gpuPlanarRegionExtraction.getParameters().getPrincipalOffsetXPixels(),
                                                               gpuPlanarRegionExtraction.getParameters().getPrincipalOffsetYPixels(),
                                                               gpuPlanarRegionExtraction.getParameters().getFocalLengthXPixels(),
                                                               gpuPlanarRegionExtraction.getParameters().getFocalLengthYPixels());
                  tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
                  boundaryPointCloud.putVertex(tempFramePoint);
               }
            }
         }

         if (render3DGrownBoundaries.get())
         {
            for (Point3D boundaryVertex : planarRegion.getBoundaryVertices())
            {
               tempFramePoint.setIncludingFrame(cameraFrame, boundaryVertex);
               tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
               boundaryPointCloud.putVertex(tempFramePoint);
            }
         }
      }

      boundaryPointCloud.updateMeshFastest();
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
      numberOfPlanarRegionsPlot.render((float) gpuPlanarRegionExtraction.getRapidPlanarRegions().size());
      regionMaxSearchDepthPlot.render((float) gpuPlanarRegionExtraction.getRegionMaxSearchDepth());
      numberOfBoundaryVerticesPlot.render((float) gpuPlanarRegionExtraction.getNumberOfBoundaryPatchesInWholeImage());
      boundaryMaxSearchDepthPlot.render((float) gpuPlanarRegionExtraction.getBoundaryMaxSearchDepth());

      boolean anyParameterChanged = gpuRegionParametersTuner.renderImGuiWidgets();

      if (anyParameterChanged && gpuRegionParametersTuner.changed(GPUPlanarRegionExtractionParameters.patchSize))
      {
         if (gpuPlanarRegionExtraction.getParameters().getPatchSize() != appliedPatchSize.get()
          && imageWidth % gpuPlanarRegionExtraction.getParameters().getPatchSize() == 0
          && imageHeight % gpuPlanarRegionExtraction.getParameters().getPatchSize() == 0)
         {
            appliedPatchSize.set(gpuPlanarRegionExtraction.getParameters().getPatchSize());
            gpuPlanarRegionExtraction.setPatchSizeChanged(true);
         }
      }
      svdDurationPlot.render((float) gpuPlanarRegionExtraction.getMaxSVDSolveTime());
      ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      ImGui.checkbox(labels.get("Draw boundaries"), drawBoundaries);
      ImGui.checkbox(labels.get("Render 3D planar regions"), render3DPlanarRegions);
      ImGui.checkbox(labels.get("Render 3D height map"), render3DHeightMap);
      ImGui.checkbox(labels.get("Render 3D boundaries"), render3DBoundaries);
      ImGui.checkbox(labels.get("Render 3D grown boundaries"), render3DGrownBoundaries);

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
      appliedPatchSize.set(parameters.getPatchSize());

      ConcaveHullFactoryParameters concaveHullFactoryParameters = gpuPlanarRegionExtraction.getConcaveHullFactoryParameters();
      edgeLengthTresholdSlider.set((float) concaveHullFactoryParameters.getEdgeLengthThreshold());
      triangulationToleranceSlider.set((float) concaveHullFactoryParameters.getTriangulationTolerance());
      maxNumberOfIterationsSlider.set(concaveHullFactoryParameters.getMaxNumberOfIterations());
      removeAllTrianglesWithTwoBorderEdgesChecked.set(concaveHullFactoryParameters.getRemoveAllTrianglesWithTwoBorderEdges());
      allowSplittingConcaveHullChecked.set(concaveHullFactoryParameters.getAllowSplittingConcaveHull());

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
      parameters.setPatchSize(appliedPatchSize.get());

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
      latestRenderables = renderables;
      latestPool = pool;

      if (render3DPlanarRegions.get())
         planarRegionsGraphic.getRenderables(renderables, pool);
      if (render3DGrownBoundaries.get() || render3DBoundaries.get())
         boundaryPointCloudSwap.accessOnHighPriorityThread();
      if (render3DHeightMap.get())
         heightMapGraphic.getRenderables(renderables, pool);
   }

   private void boundaryPointCloudUpdateOnHighPriorityThread(RDXPointCloudRenderer boundaryPointCloud)
   {
      boundaryPointCloud.getRenderables(latestRenderables, latestPool);
   }

   public void destroy()
   {
      if (pointCloudRendererShutdown != null)
         pointCloudRendererShutdown.run();
      gpuPlanarRegionExtraction.destroy();
      simpleGPUHeightMapUpdater.destroy();

      if (planarRegionsGraphic != null)
         planarRegionsGraphic.destroy();
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
