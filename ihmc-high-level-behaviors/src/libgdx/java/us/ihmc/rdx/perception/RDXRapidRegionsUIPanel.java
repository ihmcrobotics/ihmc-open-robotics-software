package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsCustomizer;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.rapidRegions.RapidRegionsDebutOutputGenerator;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImGuiVideoPanel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.utilities.ros.RosTools;

public class RDXRapidRegionsUIPanel implements RenderableProvider
{

   private RDXPlanarRegionsGraphic planarRegionsGraphic;
   private ModelInstance sensorFrameGraphic;
   private final FramePose3D framePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private RapidPlanarRegionsCustomizer rapidPlanarRegionsCustomizer;
   private RapidRegionsDebutOutputGenerator rapidRegionsDebutOutputGenerator;

   private ImGuiStoredPropertySetTuner gpuRegionParametersTuner;
   private ImGuiStoredPropertySetTuner polygonizerParametersTuner;
   private ImGuiStoredPropertySetTuner concaveHullParametersTuner;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private final ImBoolean drawPatches = new ImBoolean(true);
   private final ImBoolean drawBoundaries = new ImBoolean(true);
   private final ImBoolean render3DPlanarRegions = new ImBoolean(true);
   private final ImBoolean renderPointCloud = new ImBoolean(true);

   private ImGuiPlot wholeAlgorithmDurationPlot;
   private ImGuiPlot numberOfPlanarRegionsPlot;
   private ImGuiPlot regionMaxSearchDepthPlot;
   private ImGuiPlot boundaryMaxSearchDepthPlot;
   private ImGuiPlot svdDurationPlot;
   private ImGuiPlot gpuDurationPlot;
   private ImGuiPlot depthFirstSearchDurationPlot;
   private ImGuiPlot planarRegionCustomizationDurationPlot;

   private ImGuiPanel imguiPanel;
   private RDXCVImagePanel blurredDepthPanel;
   private RDXCVImagePanel filteredDepthPanel;
   private RDXCVImagePanel nxImagePanel;
   private RDXCVImagePanel nyImagePanel;
   private RDXCVImagePanel nzImagePanel;
   private RDXCVImagePanel gxImagePanel;
   private RDXCVImagePanel gyImagePanel;
   private RDXCVImagePanel gzImagePanel;
   private RDXImagePanel debugExtractionPanel;

   private int patchImageWidth = 0;
   private int patchImageHeight = 0;
   private int imageWidth = 0;
   private int imageHeight = 0;

   public void create(RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor)
   {
      this.rapidPlanarRegionsExtractor = rapidPlanarRegionsExtractor;
      this.rapidPlanarRegionsCustomizer = rapidPlanarRegionsExtractor.getRapidPlanarRegionsCustomizer();
      this.rapidRegionsDebutOutputGenerator = rapidPlanarRegionsExtractor.getDebugger();

      patchImageWidth = rapidPlanarRegionsExtractor.getPatchImageWidth();
      patchImageHeight = rapidPlanarRegionsExtractor.getPatchImageHeight();
      imageWidth = rapidPlanarRegionsExtractor.getImageWidth();
      imageHeight = rapidPlanarRegionsExtractor.getImageHeight();

      gpuRegionParametersTuner = new ImGuiStoredPropertySetTuner(rapidPlanarRegionsExtractor.getParameters().getTitle());
      gpuRegionParametersTuner.create(rapidPlanarRegionsExtractor.getParameters());

      polygonizerParametersTuner = new ImGuiStoredPropertySetTuner(rapidPlanarRegionsCustomizer.getPolygonizerParameters().getTitle());
      polygonizerParametersTuner.create(rapidPlanarRegionsCustomizer.getPolygonizerParameters(), true);

      concaveHullParametersTuner = new ImGuiStoredPropertySetTuner(rapidPlanarRegionsCustomizer.getConcaveHullFactoryParameters().getTitle());
      concaveHullParametersTuner.create(rapidPlanarRegionsCustomizer.getConcaveHullFactoryParameters(), true);

      imguiPanel = new ImGuiPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      blurredDepthPanel = new RDXCVImagePanel("Blurred Depth", imageWidth, imageHeight, ImGuiVideoPanel.FLIP_Y);
      filteredDepthPanel = new RDXCVImagePanel("Filtered Depth", imageWidth, imageHeight, ImGuiVideoPanel.FLIP_Y);

      nxImagePanel = new RDXCVImagePanel("Nx Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      nyImagePanel = new RDXCVImagePanel("Ny Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      nzImagePanel = new RDXCVImagePanel("Nz Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      gxImagePanel = new RDXCVImagePanel("Gx Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      gyImagePanel = new RDXCVImagePanel("Gy Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      gzImagePanel = new RDXCVImagePanel("Gz Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      debugExtractionPanel = new RDXImagePanel("Planar Region Extraction Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);

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
      boundaryMaxSearchDepthPlot = new ImGuiPlot(labels.get("Boundary max search depth"), 1000, 300, 50);
      svdDurationPlot = new ImGuiPlot(labels.get("SVD duration"), 1000, 300, 50);
      wholeAlgorithmDurationPlot = new ImGuiPlot(labels.get("Whole algorithm duration"), 1000, 300, 50);
      gpuDurationPlot = new ImGuiPlot(labels.get("GPU processing duration"), 1000, 300, 50);
      depthFirstSearchDurationPlot = new ImGuiPlot(labels.get("Depth first searching duration"), 1000, 300, 50);
      planarRegionCustomizationDurationPlot = new ImGuiPlot(labels.get("Region customization duration"), 1000, 300, 50);

      planarRegionsGraphic = new RDXPlanarRegionsGraphic();
      sensorFrameGraphic = RDXModelBuilder.createCoordinateFrameInstance(0.3);
   }

   public void render()
   {
      nxImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getCurrentFeatureGrid().getNxImage().getBytedecoOpenCVMat());
      nyImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getCurrentFeatureGrid().getNyImage().getBytedecoOpenCVMat());
      nzImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getCurrentFeatureGrid().getNzImage().getBytedecoOpenCVMat());
      gxImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getCurrentFeatureGrid().getCxImage().getBytedecoOpenCVMat());
      gyImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getCurrentFeatureGrid().getCyImage().getBytedecoOpenCVMat());
      gzImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getCurrentFeatureGrid().getCzImage().getBytedecoOpenCVMat());

      debugExtractionPanel.displayByte(rapidRegionsDebutOutputGenerator.getDebugImage());
   }

   public void render3DGraphics(PlanarRegionsList planarRegions, ReferenceFrame cameraFrame)
   {
      framePose.setToZero(cameraFrame);
      framePose.changeFrame(ReferenceFrame.getWorldFrame());
      LibGDXTools.toLibGDX(framePose, tempTransform, sensorFrameGraphic.transform);

      LogTools.info("Sensor Transform: {}", cameraFrame.getTransformToWorldFrame());
      LogTools.info("Frame Pose Transform: {}", framePose);
      LogTools.info("Sensor Graphic Transform: {}", sensorFrameGraphic.transform);


      planarRegionsGraphic.generateMeshes(planarRegions);
      planarRegionsGraphic.update();
   }

   public void renderImGuiWidgets()
   {
      int imageWidth = rapidPlanarRegionsExtractor.getImageWidth();
      int imageHeight = rapidPlanarRegionsExtractor.getImageHeight();

      ImGui.text("Input image dimensions: " + imageWidth + " x " + imageHeight);
      ImGui.checkbox(labels.get("Enabled"), enabled);

      wholeAlgorithmDurationPlot.render(rapidPlanarRegionsExtractor.getWholeAlgorithmDurationStopwatch().totalElapsed());
      gpuDurationPlot.render(rapidPlanarRegionsExtractor.getGpuDurationStopwatch().totalElapsed());
      depthFirstSearchDurationPlot.render(rapidPlanarRegionsExtractor.getDepthFirstSearchDurationStopwatch().totalElapsed());
      planarRegionCustomizationDurationPlot.render(rapidPlanarRegionsCustomizer.getStopWatch().totalElapsed());

      numberOfPlanarRegionsPlot.render((float) rapidPlanarRegionsExtractor.getGPUPlanarRegions().size());
      regionMaxSearchDepthPlot.render((float) rapidPlanarRegionsExtractor.getRegionMaxSearchDepth());
      boundaryMaxSearchDepthPlot.render((float) rapidPlanarRegionsExtractor.getBoundaryMaxSearchDepth());

      boolean anyParameterChanged = gpuRegionParametersTuner.renderImGuiWidgets();
      anyParameterChanged |= polygonizerParametersTuner.renderImGuiWidgets();
      anyParameterChanged |= concaveHullParametersTuner.renderImGuiWidgets();

      svdDurationPlot.render((float) rapidPlanarRegionsExtractor.getMaxSVDSolveTime());
      ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      ImGui.checkbox(labels.get("Draw boundaries"), drawBoundaries);
      ImGui.checkbox(labels.get("Render 3D planar regions"), render3DPlanarRegions);
      ImGui.checkbox(labels.get("Render Point Cloud"), renderPointCloud);
   }

   public void destroy()
   {
      planarRegionsGraphic.destroy();
      rapidPlanarRegionsExtractor.destroy();
      // TODO: Destroy the rest
   }

   public ImGuiPanel getPanel()
   {
      return imguiPanel;
   }

   public ImBoolean getEnabled()
   {
      return enabled;
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
      sensorFrameGraphic.getRenderables(renderables, pool);
      planarRegionsGraphic.getRenderables(renderables, pool);
   }
}
