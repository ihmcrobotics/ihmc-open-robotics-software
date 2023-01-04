package us.ihmc.rdx.perception;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsCustomizer;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.rapidRegions.RapidRegionsDebutOutputGenerator;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImGuiVideoPanel;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;

public class RDXRapidRegionsUIPanel
{
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
      planarRegionCustomizationDurationPlot = new ImGuiPlot(labels.get("Planar region segmentation duration"), 1000, 300, 50);

   }

   public void render()
   {
      nxImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getNxImage().getBytedecoOpenCVMat());
      nyImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getNyImage().getBytedecoOpenCVMat());
      nzImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getNzImage().getBytedecoOpenCVMat());
      gxImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getCxImage().getBytedecoOpenCVMat());
      gyImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getCyImage().getBytedecoOpenCVMat());
      gzImagePanel.drawDepthImage(rapidPlanarRegionsExtractor.getCzImage().getBytedecoOpenCVMat());

      debugExtractionPanel.displayByte(rapidRegionsDebutOutputGenerator.getDebugImage());
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

}
