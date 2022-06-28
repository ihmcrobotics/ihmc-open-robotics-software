package us.ihmc.gdx.perception;

import controller_msgs.msg.dds.StoredPropertySetMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionComms;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.graphics.live.GDXROS1VideoVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.utilities.ros.ROS1Helper;

public class GDXRemoteGPUPlanarRegionExtractionUI
{
   private final GPUPlanarRegionExtractionParameters gpuRegionParameters = new GPUPlanarRegionExtractionParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final GDXROS1VideoVisualizer debugExtractionPanel;
   private final ImGuiPanel panel = new ImGuiPanel("GPU Planar Regions", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat mergeDistanceThreshold = new ImFloat();
   private final ImFloat mergeAngularThreshold = new ImFloat();
   private final ImFloat filterDisparityThreshold = new ImFloat();
   private final ImInt patchSize = new ImInt();
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
   private final TypedNotification<StoredPropertySetMessage> gpuRegionParametersROS2Notification = new TypedNotification<>();
   private final TypedNotification<StoredPropertySetMessage> polygonizerParametersROS2Notification = new TypedNotification<>();
   private final TypedNotification<StoredPropertySetMessage> concaveHullFactoryParametersROS2Notification = new TypedNotification<>();
   private boolean gpuRegionParametersHaveBeenReceived = false;
   private boolean polygonizerParametersHaveBeenReceived = false;
   private boolean concaveHullFactoryParametersHaveBeenReceived = false;
   private ROS1Helper ros1Helper;
   private ROS2Helper ros2Helper;

   public GDXRemoteGPUPlanarRegionExtractionUI(ROS1Helper ros1Helper, ROS2Helper ros2Helper)
   {
      this.ros1Helper = ros1Helper;
      this.ros2Helper = ros2Helper;

      debugExtractionPanel = new GDXROS1VideoVisualizer("GPU Planar Regions Debug Image", GPUPlanarRegionExtractionComms.DEBUG_EXTRACTION_IMAGE, true);
      debugExtractionPanel.create();
      panel.addChild(debugExtractionPanel.getPanel());

      ros2Helper.subscribeViaCallback(GPUPlanarRegionExtractionComms.PARAMETERS_OUTPUT, gpuRegionParametersROS2Notification::set);
      ros2Helper.subscribeViaCallback(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS_OUTPUT, polygonizerParametersROS2Notification::set);
      ros2Helper.subscribeViaCallback(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS_OUTPUT, concaveHullFactoryParametersROS2Notification::set);
   }

   public void update()
   {
      debugExtractionPanel.updateSubscribers(ros1Helper);
      debugExtractionPanel.update();
   }

   public void renderImGuiWidgets()
   {
      debugExtractionPanel.renderImGuiWidgets();

      if (ImGui.button("Update parameters from remote"))
      {
         gpuRegionParametersHaveBeenReceived = false;
         polygonizerParametersHaveBeenReceived = false;
         concaveHullFactoryParametersHaveBeenReceived = false;
      }

      if (ImGui.button("Reconnect remote ROS 1 node"))
      {
         ros2Helper.publish(GPUPlanarRegionExtractionComms.RECONNECT_ROS1_NODE);
      }

      ImGui.separator();

      if (!gpuRegionParametersHaveBeenReceived && gpuRegionParametersROS2Notification.poll())
      {
         StoredPropertySetMessageTools.copyToStoredPropertySet(gpuRegionParametersROS2Notification.read(),
                                                               gpuRegionParameters,
                                                               () -> LogTools.info("Updating GPU planar regions parameters from remote."));
         setGPUImGuiWidgetsFromParameters();
         gpuRegionParametersHaveBeenReceived = true;
      }

      ImGui.text("GPU Planar Regions Parameters");
      if (gpuRegionParametersHaveBeenReceived)
      {
         renderGPUParameterWidgets();
      }
      else
      {
         ImGui.text("Waiting for initial values from remote...");
      }

      ImGui.separator();

      if (!polygonizerParametersHaveBeenReceived && polygonizerParametersROS2Notification.poll())
      {
         StoredPropertySetMessageTools.copyToStoredPropertySet(polygonizerParametersROS2Notification.read(),
                                                               polygonizerParameters,
                                                               () -> LogTools.info("Updating polygonizer parameters from remote."));
         setPolygonizerImGuiWidgetsFromParameters();
         polygonizerParametersHaveBeenReceived = true;
      }

      ImGui.text("Polygonizer Parameters");
      if (polygonizerParametersHaveBeenReceived)
      {
         renderPolygonizerParameterWidgets();
      }
      else
      {
         ImGui.text("Waiting for initial values from remote...");
      }

      ImGui.separator();

      if (!concaveHullFactoryParametersHaveBeenReceived && concaveHullFactoryParametersROS2Notification.poll())
      {
         StoredPropertySetMessageTools.copyToStoredPropertySet(concaveHullFactoryParametersROS2Notification.read(),
                                                               concaveHullFactoryParameters,
                                                               () -> LogTools.info("Updating concave hull factory parameters from remote."));
         setConcaveHullFactoryImGuiWidgetsFromParameters();
         concaveHullFactoryParametersHaveBeenReceived = true;
      }

      ImGui.text("Concave Hull Factory Parameters");
      if (concaveHullFactoryParametersHaveBeenReceived)
      {
         renderConcaveHullFactoryParameterWidgets();
      }
      else
      {
         ImGui.text("Waiting for initial values from remote...");
      }
   }

   private void renderGPUParameterWidgets()
   {
      boolean anyChanged = false;
      anyChanged |= ImGui.checkbox(labels.get("Early gaussian blur"), earlyGaussianBlur);
      anyChanged |= ImGui.sliderInt(labels.get("Gaussian size"), gaussianSize.getData(), 1, 20);
      anyChanged |= ImGui.sliderFloat(labels.get("Gaussian sigma"), gaussianSigma.getData(), 0.23f, 10.0f);
      anyChanged |= ImGui.sliderInt(labels.get("Patch size"), patchSize.getData(), 2, 20);
      anyChanged |= ImGui.sliderInt(labels.get("Dead pixel filter patch size"), deadPixelFilterPatchSize.getData(), 1, 20);
      anyChanged |= ImGui.checkbox(labels.get("Use filtered image"), useFilteredImage);
      anyChanged |= ImGui.sliderFloat(labels.get("Merge distance threshold"), mergeDistanceThreshold.getData(), 0.0f, 0.1f);
      anyChanged |= ImGui.sliderFloat(labels.get("Merge angular threshold"), mergeAngularThreshold.getData(), 0.0f, 1.0f);
      anyChanged |= ImGui.sliderInt(labels.get("Search depth limit"), searchDepthLimit.getData(), 1, 50000);
      anyChanged |= ImGui.sliderInt(labels.get("Region min patches"), regionMinPatches.getData(), 1, 3000);
      anyChanged |= ImGui.sliderInt(labels.get("Boundary min patches"), boundaryMinPatches.getData(), 1, 1000);
      anyChanged |= ImGui.inputFloat(labels.get("Filter disparity threshold"), filterDisparityThreshold);
      anyChanged |= ImGui.sliderFloat(labels.get("Region growth factor"), regionGrowthFactor.getData(), 0.005f, 0.1f);
      anyChanged |= ImGui.checkbox(labels.get("Use SVD normals"), useSVDNormals);
      anyChanged |= ImGui.sliderInt(labels.get("SVD reduction factor"), svdReductionFactor.getData(), 1, 100);
      anyChanged |= ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      anyChanged |= ImGui.checkbox(labels.get("Draw boundaries"), drawBoundaries);
      anyChanged |= ImGui.checkbox(labels.get("Render 3D planar regions"), render3DPlanarRegions);
      anyChanged |= ImGui.checkbox(labels.get("Render 3D boundaries"), render3DBoundaries);
      anyChanged |= ImGui.checkbox(labels.get("Render 3D grown boundaries"), render3DGrownBoundaries);
      anyChanged |= ImGui.sliderFloat(labels.get("Focal length X (px)"), focalLengthXPixels.getData(), -1000.0f, 1000.0f);
      anyChanged |= ImGui.sliderFloat(labels.get("Focal length Y (px)"), focalLengthYPixels.getData(), -1000.0f, 1000.0f);
      anyChanged |= ImGui.inputFloat(labels.get("Principal offset X (px)"), principalOffsetXPixels);
      anyChanged |= ImGui.inputFloat(labels.get("Principal offset Y (px)"), principalOffsetYPixels);

      if (anyChanged)
      {
         setGPUParametersFromImGuiWidgets();
         ros2Helper.publish(GPUPlanarRegionExtractionComms.PARAMETERS_INPUT, StoredPropertySetMessageTools.newMessage(gpuRegionParameters));
      }
   }

   private void renderPolygonizerParameterWidgets()
   {
      boolean anyChanged = false;
      anyChanged |= ImGui.sliderFloat("Concave Hull Threshold", concaveHullThresholdSlider.getData(), 0, 1);
      anyChanged |= ImGui.sliderFloat("Shallow Angle Threshold", shallowAngleThresholdSlider.getData(), 0, 2.0f * (float) Math.PI);
      anyChanged |= ImGui.sliderFloat("Peak Angle Threshold", peakAngleThresholdSlider.getData(), 0, 2.0f * (float) Math.PI);
      anyChanged |= ImGui.sliderFloat("Length Threshold", lengthThresholdSlider.getData(), 0, 1);
      anyChanged |= ImGui.sliderFloat("Depth Threshold", depthThresholdSlider.getData(), 0, 1);
      anyChanged |= ImGui.sliderInt("Min Number of Nodes", minNumberOfNodesSlider.getData(), 0, 100);
      anyChanged |= ImGui.checkbox("Cut Narrow Passage", cutNarrowPassageChecked);

      if (anyChanged)
      {
         setPolygonizerParametersFromImGuiWidgets();
         ros2Helper.publish(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS_INPUT, StoredPropertySetMessageTools.newMessage(polygonizerParameters));
      }
   }

   private void renderConcaveHullFactoryParameterWidgets()
   {
      boolean anyChanged = false;
      ImGui.sliderFloat("Edge Length Threshold", edgeLengthTresholdSlider.getData(), 0, 0.5f);
      ImGui.sliderFloat("Triangulation Tolerance", triangulationToleranceSlider.getData(), 0, 0.3f);
      ImGui.sliderInt("Max Number of Iterations", maxNumberOfIterationsSlider.getData(), 2000, 6000);
      ImGui.checkbox("Remove Degenerate Triangles", removeAllTrianglesWithTwoBorderEdgesChecked);
      ImGui.checkbox("Split Concave Hull", allowSplittingConcaveHullChecked);

      if (anyChanged)
      {
         setConcaveHullParametersFromImGuiWidgets();
         ros2Helper.publish(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS_INPUT,
                            StoredPropertySetMessageTools.newMessage(concaveHullFactoryParameters));
      }
   }

   private void setGPUParametersFromImGuiWidgets()
   {
      gpuRegionParameters.setMergeDistanceThreshold(mergeDistanceThreshold.get());
      gpuRegionParameters.setMergeAngularThreshold(mergeAngularThreshold.get());
      gpuRegionParameters.setFilterDisparityThreshold(filterDisparityThreshold.get());
      gpuRegionParameters.setPatchSize(patchSize.get());
      gpuRegionParameters.setDeadPixelFilterPatchSize(deadPixelFilterPatchSize.get());
      gpuRegionParameters.setFocalLengthXPixels(focalLengthXPixels.get());
      gpuRegionParameters.setFocalLengthYPixels(focalLengthYPixels.get());
      gpuRegionParameters.setPrincipalOffsetXPixels(principalOffsetXPixels.get());
      gpuRegionParameters.setPrincipalOffsetYPixels(principalOffsetYPixels.get());
      gpuRegionParameters.setEarlyGaussianBlur(earlyGaussianBlur.get());
      gpuRegionParameters.setUseFilteredImage(useFilteredImage.get());
      gpuRegionParameters.setUseSVDNormals(useSVDNormals.get());
      gpuRegionParameters.setSVDReductionFactor(svdReductionFactor.get());
      gpuRegionParameters.setGaussianSize(gaussianSize.get());
      gpuRegionParameters.setGaussianSigma(gaussianSigma.get());
      gpuRegionParameters.setSearchDepthLimit(searchDepthLimit.get());
      gpuRegionParameters.setRegionMinPatches(regionMinPatches.get());
      gpuRegionParameters.setBoundaryMinPatches(boundaryMinPatches.get());
      gpuRegionParameters.setRegionGrowthFactor(regionGrowthFactor.get());
   }

   private void setPolygonizerParametersFromImGuiWidgets()
   {
      polygonizerParameters.setConcaveHullThreshold(concaveHullThresholdSlider.get());
      polygonizerParameters.setShallowAngleThreshold(shallowAngleThresholdSlider.get());
      polygonizerParameters.setPeakAngleThreshold(peakAngleThresholdSlider.get());
      polygonizerParameters.setLengthThreshold(lengthThresholdSlider.get());
      polygonizerParameters.setDepthThreshold(depthThresholdSlider.get());
      polygonizerParameters.setMinNumberOfNodes(minNumberOfNodesSlider.get());
      polygonizerParameters.setCutNarrowPassage(cutNarrowPassageChecked.get());
   }

   private void setConcaveHullParametersFromImGuiWidgets()
   {
      concaveHullFactoryParameters.setEdgeLengthThreshold(edgeLengthTresholdSlider.get());
      concaveHullFactoryParameters.setTriangulationTolerance(triangulationToleranceSlider.get());
      concaveHullFactoryParameters.setMaxNumberOfIterations(maxNumberOfIterationsSlider.get());
      concaveHullFactoryParameters.setRemoveAllTrianglesWithTwoBorderEdges(removeAllTrianglesWithTwoBorderEdgesChecked.get());
      concaveHullFactoryParameters.setAllowSplittingConcaveHull(allowSplittingConcaveHullChecked.get());
   }

   public void setGPUImGuiWidgetsFromParameters()
   {
      mergeDistanceThreshold.set((float) gpuRegionParameters.getMergeDistanceThreshold());
      mergeAngularThreshold.set((float) gpuRegionParameters.getMergeAngularThreshold());
      filterDisparityThreshold.set((float) gpuRegionParameters.getFilterDisparityThreshold());
      patchSize.set(gpuRegionParameters.getPatchSize());
      deadPixelFilterPatchSize.set(gpuRegionParameters.getDeadPixelFilterPatchSize());
      focalLengthXPixels.set((float) gpuRegionParameters.getFocalLengthXPixels());
      focalLengthYPixels.set((float) gpuRegionParameters.getFocalLengthYPixels());
      principalOffsetXPixels.set((float) gpuRegionParameters.getPrincipalOffsetXPixels());
      principalOffsetYPixels.set((float) gpuRegionParameters.getPrincipalOffsetYPixels());
      earlyGaussianBlur.set(gpuRegionParameters.getEarlyGaussianBlur());
      useFilteredImage.set(gpuRegionParameters.getUseFilteredImage());
      useSVDNormals.set(gpuRegionParameters.getUseSVDNormals());
      svdReductionFactor.set(gpuRegionParameters.getSVDReductionFactor());
      gaussianSize.set(gpuRegionParameters.getGaussianSize());
      gaussianSigma.set((float) gpuRegionParameters.getGaussianSigma());
      searchDepthLimit.set(gpuRegionParameters.getSearchDepthLimit());
      regionMinPatches.set(gpuRegionParameters.getRegionMinPatches());
      boundaryMinPatches.set(gpuRegionParameters.getBoundaryMinPatches());
      regionGrowthFactor.set((float) gpuRegionParameters.getRegionGrowthFactor());
   }

   public void setPolygonizerImGuiWidgetsFromParameters()
   {
      concaveHullThresholdSlider.set((float) polygonizerParameters.getConcaveHullThreshold());
      shallowAngleThresholdSlider.set((float) polygonizerParameters.getShallowAngleThreshold());
      peakAngleThresholdSlider.set((float) polygonizerParameters.getPeakAngleThreshold());
      lengthThresholdSlider.set((float) polygonizerParameters.getLengthThreshold());
      depthThresholdSlider.set((float) polygonizerParameters.getDepthThreshold());
      minNumberOfNodesSlider.set(polygonizerParameters.getMinNumberOfNodes());
      cutNarrowPassageChecked.set(polygonizerParameters.getCutNarrowPassage());
   }

   public void setConcaveHullFactoryImGuiWidgetsFromParameters()
   {
      edgeLengthTresholdSlider.set((float) concaveHullFactoryParameters.getEdgeLengthThreshold());
      triangulationToleranceSlider.set((float) concaveHullFactoryParameters.getTriangulationTolerance());
      maxNumberOfIterationsSlider.set(concaveHullFactoryParameters.getMaxNumberOfIterations());
      removeAllTrianglesWithTwoBorderEdgesChecked.set(concaveHullFactoryParameters.doRemoveAllTrianglesWithTwoBorderEdges());
      allowSplittingConcaveHullChecked.set(concaveHullFactoryParameters.isSplittingConcaveHullAllowed());
   }

   public void destroy()
   {
      debugExtractionPanel.destroy();
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
