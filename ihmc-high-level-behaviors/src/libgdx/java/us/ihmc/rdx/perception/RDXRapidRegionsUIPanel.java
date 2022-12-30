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
import us.ihmc.avatar.gpuPlanarRegions.*;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapParameters;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapUpdater;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImGuiVideoPanel;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.ui.graphics.live.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.visualizers.RDXHeightMapGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.robotics.perception.ProjectionTools;
import us.ihmc.tools.thread.ZeroCopySwapReference;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.nio.ByteBuffer;

public class RDXRapidRegionsUIPanel
{
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private RapidPlanarRegionsCustomizer rapidPlanarRegionsCustomizer;
   private RapidRegionsDebutOutputGenerator rapidRegionsDebutOutputGenerator;

   private ImGuiStoredPropertySetTuner gpuRegionParametersTuner;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private final ImBoolean drawPatches = new ImBoolean(true);
   private final ImBoolean drawBoundaries = new ImBoolean(true);
   private final ImBoolean render3DPlanarRegions = new ImBoolean(true);
   private final ImBoolean render3DBoundaries = new ImBoolean(true);
   private final ImBoolean render3DGrownBoundaries = new ImBoolean(true);

   private ImGuiPlot wholeAlgorithmDurationPlot;
   private ImGuiPlot numberOfPlanarRegionsPlot;
   private ImGuiPlot regionMaxSearchDepthPlot;
   private ImGuiPlot boundaryMaxSearchDepthPlot;
   private ImGuiPlot svdDurationPlot;
   private ImGuiPlot gpuDurationPlot;
   private ImGuiPlot depthFirstSearchDurationPlot;
   private ImGuiPlot planarRegionsSegmentationDurationPlot;

   private ImGuiPanel imguiPanel;
   private RDXImagePanel blurredDepthPanel;
   private RDXImagePanel filteredDepthPanel;
   private RDXImagePanel nxImagePanel;
   private RDXImagePanel nyImagePanel;
   private RDXImagePanel nzImagePanel;
   private RDXImagePanel gxImagePanel;
   private RDXImagePanel gyImagePanel;
   private RDXImagePanel gzImagePanel;
   private RDXImagePanel debugExtractionPanel;

   private int patchImageWidth = 0;
   private int patchImageHeight = 0;
   private int imageWidth = 0;
   private int imageHeight = 0;

   private RDXPlanarRegionsGraphic planarRegionsGraphic;

   public void create(RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor,
                      RapidPlanarRegionsCustomizer rapidPlanarRegionsCustomizer)
   {
      this.rapidPlanarRegionsExtractor = rapidPlanarRegionsExtractor;
      this.rapidPlanarRegionsCustomizer = rapidPlanarRegionsCustomizer;
      this.rapidRegionsDebutOutputGenerator = rapidPlanarRegionsExtractor.getDebugger();

      patchImageWidth = rapidPlanarRegionsExtractor.getPatchImageWidth();
      patchImageHeight = rapidPlanarRegionsExtractor.getPatchImageHeight();
      imageWidth = rapidPlanarRegionsExtractor.getImageWidth();
      imageHeight = rapidPlanarRegionsExtractor.getImageHeight();

      gpuRegionParametersTuner = new ImGuiStoredPropertySetTuner(rapidPlanarRegionsExtractor.getParameters().getTitle());
      gpuRegionParametersTuner.create(rapidPlanarRegionsExtractor.getParameters());

      imguiPanel = new ImGuiPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      blurredDepthPanel = new RDXImagePanel("Blurred Depth", imageWidth, imageHeight, ImGuiVideoPanel.FLIP_Y);
      filteredDepthPanel = new RDXImagePanel("Filtered Depth", imageWidth, imageHeight, ImGuiVideoPanel.FLIP_Y);

      nxImagePanel = new RDXImagePanel("Nx Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      nyImagePanel = new RDXImagePanel("Ny Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      nzImagePanel = new RDXImagePanel("Nz Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      gxImagePanel = new RDXImagePanel("Gx Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      gyImagePanel = new RDXImagePanel("Gy Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
      gzImagePanel = new RDXImagePanel("Gz Image", patchImageWidth, patchImageHeight, ImGuiVideoPanel.FLIP_Y);
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
      planarRegionsSegmentationDurationPlot = new ImGuiPlot(labels.get("Planar region segmentation duration"), 1000, 300, 50);

      planarRegionsGraphic = new RDXPlanarRegionsGraphic();
   }

   public void render()
   {
      nxImagePanel.displayRaw(rapidPlanarRegionsExtractor.getNxImage().getBytedecoOpenCVMat());
      nyImagePanel.displayRaw(rapidPlanarRegionsExtractor.getNyImage().getBytedecoOpenCVMat());
      nzImagePanel.displayRaw(rapidPlanarRegionsExtractor.getNzImage().getBytedecoOpenCVMat());
      gxImagePanel.displayRaw(rapidPlanarRegionsExtractor.getCxImage().getBytedecoOpenCVMat());
      gyImagePanel.displayRaw(rapidPlanarRegionsExtractor.getCyImage().getBytedecoOpenCVMat());
      gzImagePanel.displayRaw(rapidPlanarRegionsExtractor.getCzImage().getBytedecoOpenCVMat());
      debugExtractionPanel.display();

      if (!render3DPlanarRegions.get())
         return;

      planarRegionsGraphic.generateMeshes(rapidPlanarRegionsExtractor.getPlanarRegionsList());
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
      planarRegionsSegmentationDurationPlot.render(rapidPlanarRegionsExtractor.getPlanarRegionsSegmentationDurationStopwatch().totalElapsed());

      numberOfPlanarRegionsPlot.render((float) rapidPlanarRegionsExtractor.getGPUPlanarRegions().size());
      regionMaxSearchDepthPlot.render((float) rapidPlanarRegionsExtractor.getRegionMaxSearchDepth());
      boundaryMaxSearchDepthPlot.render((float) rapidPlanarRegionsExtractor.getBoundaryMaxSearchDepth());

      boolean anyParameterChanged = gpuRegionParametersTuner.renderImGuiWidgets();

      svdDurationPlot.render((float) rapidPlanarRegionsExtractor.getMaxSVDSolveTime());
      ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      ImGui.checkbox(labels.get("Draw boundaries"), drawBoundaries);
      ImGui.checkbox(labels.get("Render 3D planar regions"), render3DPlanarRegions);
      ImGui.checkbox(labels.get("Render 3D boundaries"), render3DBoundaries);
      ImGui.checkbox(labels.get("Render 3D grown boundaries"), render3DGrownBoundaries);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (render3DPlanarRegions.get())
         planarRegionsGraphic.getRenderables(renderables, pool);
   }

   public void destroy()
   {
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
}
