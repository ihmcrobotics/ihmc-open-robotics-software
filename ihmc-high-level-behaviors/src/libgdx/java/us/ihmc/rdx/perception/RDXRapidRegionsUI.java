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

   private RDXPanel imguiPanel;
   private RDXBytedecoImagePanel blurredDepthPanel;
   private RDXBytedecoImagePanel filteredDepthPanel;
   private RDXBytedecoImagePanel nxImagePanel;
   private RDXBytedecoImagePanel nyImagePanel;
   private RDXBytedecoImagePanel nzImagePanel;
   private RDXBytedecoImagePanel gxImagePanel;
   private RDXBytedecoImagePanel gyImagePanel;
   private RDXBytedecoImagePanel gzImagePanel;
   private RDXMatImagePanel debugExtractionPanel;

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

      gpuRegionParametersTuner = new RDXStoredPropertySetTuner(rapidPlanarRegionsExtractor.getParameters().getTitle());
      gpuRegionParametersTuner.create(rapidPlanarRegionsExtractor.getParameters());

      polygonizerParametersTuner = new RDXStoredPropertySetTuner(rapidPlanarRegionsCustomizer.getPolygonizerParameters().getTitle());
      polygonizerParametersTuner.create(rapidPlanarRegionsCustomizer.getPolygonizerParameters(), true);

      concaveHullParametersTuner = new RDXStoredPropertySetTuner(rapidPlanarRegionsCustomizer.getConcaveHullFactoryParameters().getTitle());
      concaveHullParametersTuner.create(rapidPlanarRegionsCustomizer.getConcaveHullFactoryParameters(), true);

      imguiPanel = new RDXPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      blurredDepthPanel = new RDXBytedecoImagePanel("Blurred Depth", imageWidth, imageHeight, RDXImagePanel.FLIP_Y);
      filteredDepthPanel = new RDXBytedecoImagePanel("Filtered Depth", imageWidth, imageHeight, RDXImagePanel.FLIP_Y);

      nxImagePanel = new RDXBytedecoImagePanel("Nx Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      nyImagePanel = new RDXBytedecoImagePanel("Ny Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      nzImagePanel = new RDXBytedecoImagePanel("Nz Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      gxImagePanel = new RDXBytedecoImagePanel("Gx Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      gyImagePanel = new RDXBytedecoImagePanel("Gy Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      gzImagePanel = new RDXBytedecoImagePanel("Gz Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);
      debugExtractionPanel = new RDXMatImagePanel("Planar Region Extraction Image", patchImageWidth, patchImageHeight, RDXImagePanel.FLIP_Y);

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
      int imageWidth = rapidPlanarRegionsExtractor.getImageWidth();
      int imageHeight = rapidPlanarRegionsExtractor.getImageHeight();

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
         regionMaxSearchDepthPlot.render((float) rapidPlanarRegionsExtractor.getRegionMaxSearchDepth());
         boundaryMaxSearchDepthPlot.render((float) rapidPlanarRegionsExtractor.getBoundaryMaxSearchDepth());
      }

      boolean anyParameterChanged = gpuRegionParametersTuner.renderImGuiWidgets();
      anyParameterChanged |= polygonizerParametersTuner.renderImGuiWidgets();
      anyParameterChanged |= concaveHullParametersTuner.renderImGuiWidgets();

      svdDurationPlot.render((float) rapidPlanarRegionsExtractor.getMaxSVDSolveTime());
   }

   public void destroy()
   {
      planarRegionsGraphic.destroy();
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
