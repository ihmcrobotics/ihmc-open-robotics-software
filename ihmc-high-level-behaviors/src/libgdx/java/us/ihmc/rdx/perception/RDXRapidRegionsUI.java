package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsCustomizer;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.rapidRegions.RapidPatchesDebugOutputGenerator;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class RDXRapidRegionsUI implements RenderableProvider
{
   private static final boolean RENDER_DEBUG_PLOTS = false;

   private RDXPlanarRegionsGraphic planarRegionsGraphic;
   private ModelInstance sensorFrameGraphic;
   private final FramePose3D framePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private RapidPlanarRegionsCustomizer rapidPlanarRegionsCustomizer;
   private RapidPatchesDebugOutputGenerator rapidRegionsDebutOutputGenerator;

   private RDXStoredPropertySetTuner gpuRegionParametersTuner;
   private RDXStoredPropertySetTuner polygonizerParametersTuner;
   private RDXStoredPropertySetTuner concaveHullParametersTuner;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(true);
   private final ImBoolean drawPatches = new ImBoolean(true);
   private final ImBoolean drawBoundaries = new ImBoolean(true);
   private final ImBoolean render3DPlanarRegions = new ImBoolean(false);
   private final ImBoolean renderPointCloud = new ImBoolean(false);

   private ImGuiPlot wholeAlgorithmDurationPlot;
   private ImGuiPlot numberOfPlanarRegionsPlot;
   private ImGuiPlot regionMaxSearchDepthPlot;
   private ImGuiPlot boundaryMaxSearchDepthPlot;
   private ImGuiPlot svdDurationPlot;
   private ImGuiPlot gpuDurationPlot;
   private ImGuiPlot depthFirstSearchDurationPlot;
   private ImGuiPlot planarRegionCustomizationDurationPlot;

   private RDXPanel imguiPanel;
   private RDXMatImagePanel depthPanel;
   private RDXMatImagePanel nxImagePanel;
   private RDXMatImagePanel nyImagePanel;
   private RDXMatImagePanel nzImagePanel;
   private RDXMatImagePanel gxImagePanel;
   private RDXMatImagePanel gyImagePanel;
   private RDXMatImagePanel gzImagePanel;
   private RDXMatImagePanel debugExtractionPanel;

   private int patchImageWidth = 0;
   private int patchImageHeight = 0;
   private int imageWidth = 0;
   private int imageHeight = 0;

   private Mat tempNormalized8U;
   private Mat tempRGBA;

   public void create(RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor)
   {
      this.rapidPlanarRegionsExtractor = rapidPlanarRegionsExtractor;
      this.rapidPlanarRegionsCustomizer = rapidPlanarRegionsExtractor.getRapidPlanarRegionsCustomizer();
      this.rapidRegionsDebutOutputGenerator = rapidPlanarRegionsExtractor.getDebugger();
      this.rapidRegionsDebutOutputGenerator.setEnabled(true);

      patchImageWidth = rapidPlanarRegionsExtractor.getPatchImageWidth();
      patchImageHeight = rapidPlanarRegionsExtractor.getPatchImageHeight();
      imageWidth = rapidPlanarRegionsExtractor.getCameraIntrinsics().getWidth();
      imageHeight = rapidPlanarRegionsExtractor.getCameraIntrinsics().getHeight();

      gpuRegionParametersTuner = new RDXStoredPropertySetTuner(rapidPlanarRegionsExtractor.getRapidRegionsExtractorParameters().getTitle());
      gpuRegionParametersTuner.create(rapidPlanarRegionsExtractor.getRapidRegionsExtractorParameters());

      polygonizerParametersTuner = new RDXStoredPropertySetTuner(rapidPlanarRegionsCustomizer.getPolygonizerParameters().getTitle());
      polygonizerParametersTuner.create(rapidPlanarRegionsCustomizer.getPolygonizerParameters(), true);

      concaveHullParametersTuner = new RDXStoredPropertySetTuner(rapidPlanarRegionsCustomizer.getConcaveHullFactoryParameters().getTitle());
      concaveHullParametersTuner.create(rapidPlanarRegionsCustomizer.getConcaveHullFactoryParameters(), true);

      imguiPanel = new RDXPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      depthPanel = new RDXMatImagePanel("Depth", imageWidth, imageHeight, RDXImagePanel.DO_NOT_FLIP_Y);
      nxImagePanel = new RDXMatImagePanel("Nx Image", patchImageWidth, patchImageHeight, RDXImagePanel.DO_NOT_FLIP_Y);
      nyImagePanel = new RDXMatImagePanel("Ny Image", patchImageWidth, patchImageHeight, RDXImagePanel.DO_NOT_FLIP_Y);
      nzImagePanel = new RDXMatImagePanel("Nz Image", patchImageWidth, patchImageHeight, RDXImagePanel.DO_NOT_FLIP_Y);
      gxImagePanel = new RDXMatImagePanel("Gx Image", patchImageWidth, patchImageHeight, RDXImagePanel.DO_NOT_FLIP_Y);
      gyImagePanel = new RDXMatImagePanel("Gy Image", patchImageWidth, patchImageHeight, RDXImagePanel.DO_NOT_FLIP_Y);
      gzImagePanel = new RDXMatImagePanel("Gz Image", patchImageWidth, patchImageHeight, RDXImagePanel.DO_NOT_FLIP_Y);
      debugExtractionPanel = new RDXMatImagePanel("Planar Region Extraction Image", imageWidth, imageHeight, RDXImagePanel.DO_NOT_FLIP_Y);

      imguiPanel.addChild(depthPanel.getImagePanel());
      imguiPanel.addChild(nxImagePanel.getImagePanel());
      imguiPanel.addChild(nyImagePanel.getImagePanel());
      imguiPanel.addChild(nzImagePanel.getImagePanel());
      imguiPanel.addChild(gxImagePanel.getImagePanel());
      imguiPanel.addChild(gyImagePanel.getImagePanel());
      imguiPanel.addChild(gzImagePanel.getImagePanel());
      imguiPanel.addChild(debugExtractionPanel.getImagePanel());

      numberOfPlanarRegionsPlot = new ImGuiPlot(labels.get("Number of planar regions"), 1000, 300, 50);
      regionMaxSearchDepthPlot = new ImGuiPlot(labels.get("Regions max search depth"), 1000, 300, 50);
      boundaryMaxSearchDepthPlot = new ImGuiPlot(labels.get("Boundary max search depth"), 1000, 300, 50);
      svdDurationPlot = new ImGuiPlot(labels.get("SVD duration"), 1000, 300, 50);
      wholeAlgorithmDurationPlot = new ImGuiPlot(labels.get("Whole algorithm duration"), 1000, 300, 50);
      gpuDurationPlot = new ImGuiPlot(labels.get("GPU processing duration"), 1000, 300, 50);
      depthFirstSearchDurationPlot = new ImGuiPlot(labels.get("Depth first searching duration"), 1000, 300, 50);
      planarRegionCustomizationDurationPlot = new ImGuiPlot(labels.get("Region customization duration"), 1000, 300, 50);

      planarRegionsGraphic = new RDXPlanarRegionsGraphic();
      sensorFrameGraphic = RDXModelBuilder.createCoordinateFrameInstance(0.3);

      tempNormalized8U = new Mat();
      tempRGBA = new Mat();
   }

   public void render()
   {
      displayFloatImage(rapidPlanarRegionsExtractor.getPatchNormalsXHost(), nxImagePanel);
      displayFloatImage(rapidPlanarRegionsExtractor.getPatchNormalsYHost(), nyImagePanel);
      displayFloatImage(rapidPlanarRegionsExtractor.getPatchNormalsZHost(), nzImagePanel);
      displayFloatImage(rapidPlanarRegionsExtractor.getPatchCentroidsXHost(), gxImagePanel);
      displayFloatImage(rapidPlanarRegionsExtractor.getPatchCentroidsYHost(), gyImagePanel);
      displayFloatImage(rapidPlanarRegionsExtractor.getPatchCentroidsZHost(), gzImagePanel);

      Mat debugImage = rapidRegionsDebutOutputGenerator.getDebugImage();
      if (debugExtractionPanel.getImagePanel().getIsShowing().get() && !debugImage.empty())
      {
         debugExtractionPanel.displayByte(debugImage);
      }
   }

   private void displayFloatImage(Mat floatMat, RDXMatImagePanel panel)
   {
      if (panel.getImagePanel().getIsShowing().get() && !floatMat.empty())
      {
         // Check if the mat has valid dimensions and data
         if (floatMat.rows() == 0 || floatMat.cols() == 0)
            return;

         // Find min and max values in the image
         double[] minVal = new double[1];
         double[] maxVal = new double[1];
         opencv_core.minMaxLoc(floatMat, minVal, maxVal, null, null, opencv_core.noArray());

         // Skip if min and max are both zero (uninitialized data)
         if (Math.abs(minVal[0]) < 1e-10 && Math.abs(maxVal[0]) < 1e-10)
            return;

         // Normalize float values to 0-255 range using actual min/max from the image
         // This ensures we use the full dynamic range for visualization
         double alpha = 255.0 / (maxVal[0] - minVal[0] + 1e-10); // Avoid division by zero
         double beta = -minVal[0] * alpha;
         floatMat.convertTo(tempNormalized8U, opencv_core.CV_8U, alpha, beta);

         // Convert grayscale to RGBA
         opencv_imgproc.cvtColor(tempNormalized8U, tempRGBA, opencv_imgproc.COLOR_GRAY2RGBA);

         // Display the RGBA image
         panel.displayByte(tempRGBA);
      }
   }

   public void render3DGraphics(FramePlanarRegionsList planarRegions)
   {
      if (render3DPlanarRegions.get())
      {
         framePose.setToZero(ReferenceFrame.getWorldFrame());
         framePose.set(planarRegions.getSensorToWorldFrameTransform());
         LibGDXTools.toLibGDX(framePose, tempTransform, sensorFrameGraphic.transform);

         PlanarRegionsList regionsToRender = planarRegions.getPlanarRegionsList().copy();
         regionsToRender.applyTransform(planarRegions.getSensorToWorldFrameTransform());

         planarRegionsGraphic.generateMeshes(regionsToRender);
         planarRegionsGraphic.update();
      }
   }

   public void renderImGuiWidgets()
   {
      int imageWidth = rapidPlanarRegionsExtractor.getCameraIntrinsics().getWidth();
      int imageHeight = rapidPlanarRegionsExtractor.getCameraIntrinsics().getHeight();

      ImGui.text("Input image dimensions: " + imageWidth + " x " + imageHeight);
      ImGui.checkbox(labels.get("Enabled"), enabled);
      ImGui.checkbox(labels.get("Render 3D planar regions"), render3DPlanarRegions);
      ImGui.checkbox(labels.get("Render Point Cloud"), renderPointCloud);
      ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      ImGui.checkbox(labels.get("Draw boundaries"), drawBoundaries);

      if (RENDER_DEBUG_PLOTS)
      {
         wholeAlgorithmDurationPlot.render(rapidPlanarRegionsExtractor.getWholeAlgorithmDurationStopwatch().totalElapsed());
         gpuDurationPlot.render(rapidPlanarRegionsExtractor.getGpuDurationStopwatch().totalElapsed());
         depthFirstSearchDurationPlot.render(rapidPlanarRegionsExtractor.getDepthFirstSearchDurationStopwatch().totalElapsed());
         planarRegionCustomizationDurationPlot.render(rapidPlanarRegionsCustomizer.getStopWatch().totalElapsed());

         numberOfPlanarRegionsPlot.render((float) rapidPlanarRegionsExtractor.getRapidPlanarRegions().size());
         regionMaxSearchDepthPlot.render((float) rapidPlanarRegionsExtractor.getRapidRegionsExtractorParameters().getInternalSearchDepthLimit());
         boundaryMaxSearchDepthPlot.render((float) rapidPlanarRegionsExtractor.getRapidRegionsExtractorParameters().getBoundarySearchDepthLimit());
      }

      gpuRegionParametersTuner.renderImGuiWidgets();
      polygonizerParametersTuner.renderImGuiWidgets();
      concaveHullParametersTuner.renderImGuiWidgets();

      svdDurationPlot.render((float) rapidPlanarRegionsExtractor.getMaxSVDSolveTime());
   }

   public void destroy()
   {
      planarRegionsGraphic.destroy();
      if (tempNormalized8U != null)
         tempNormalized8U.close();
      if (tempRGBA != null)
         tempRGBA.close();
   }

   public RDXPanel getPanel()
   {
      return imguiPanel;
   }

   public ImBoolean getEnabled()
   {
      return enabled;
   }

   public ImBoolean getRender3DPlanarRegionsEnabled()
   {
      return render3DPlanarRegions;
   }

   public boolean getPointCloudRenderEnabled()
   {
      return renderPointCloud.get();
   }

   public boolean get3DPlanarRegionsRenderEnabled()
   {
      return render3DPlanarRegions.get();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if(render3DPlanarRegions.get())
      {
         sensorFrameGraphic.getRenderables(renderables, pool);
         planarRegionsGraphic.getRenderables(renderables, pool);
      }
   }
}
