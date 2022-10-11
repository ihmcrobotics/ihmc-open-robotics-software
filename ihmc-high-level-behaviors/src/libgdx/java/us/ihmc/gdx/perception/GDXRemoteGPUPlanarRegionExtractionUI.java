package us.ihmc.gdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionComms;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.communication.property.StoredPropertySetROS2Input;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.graphics.live.GDXROS1VideoVisualizer;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.utilities.ros.ROS1Helper;

public class GDXRemoteGPUPlanarRegionExtractionUI
{
   private final ROS1Helper ros1Helper;
   private final ROS2Helper ros2Helper;
   private final GPUPlanarRegionExtractionParameters gpuRegionParameters = new GPUPlanarRegionExtractionParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private GDXROS1VideoVisualizer debugExtractionPanel;
   private final ImGuiPanel panel = new ImGuiPanel("GPU Planar Regions", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat edgeLengthThresholdSlider = new ImFloat();
   private final ImFloat triangulationToleranceSlider = new ImFloat();
   private final ImInt maxNumberOfIterationsSlider = new ImInt();
   private final ImBoolean allowSplittingConcaveHullChecked = new ImBoolean();
   private final ImBoolean removeAllTrianglesWithTwoBorderEdgesChecked = new ImBoolean();
   private final ImFloat concaveHullThresholdSlider = new ImFloat();
   private final ImFloat shallowAngleThresholdSlider = new ImFloat();
   private final ImFloat peakAngleThresholdSlider = new ImFloat();
   private final ImFloat lengthThresholdSlider = new ImFloat();
   private final ImFloat depthThresholdSlider = new ImFloat();
   private final ImInt minNumberOfNodesSlider = new ImInt();
   private final ImBoolean cutNarrowPassageChecked = new ImBoolean();
   private final StoredPropertySetROS2Input gpuRegionParametersROS2Input;
   private final StoredPropertySetROS2Input polygonizerParametersROS2Input;
   private final StoredPropertySetROS2Input concaveHullFactoryParametersROS2Input;
   private final ImGuiStoredPropertySetTuner gpuRegionParametersTuner;
   private boolean gpuRegionParametersChangedByImGuiUser = false;

   public GDXRemoteGPUPlanarRegionExtractionUI(ROS1Helper ros1Helper, ROS2Helper ros2Helper)
   {
      this.ros1Helper = ros1Helper;
      this.ros2Helper = ros2Helper;

      if (ros1Helper != null)
      {
         debugExtractionPanel = new GDXROS1VideoVisualizer("GPU Planar Regions Debug Image", GPUPlanarRegionExtractionComms.DEBUG_EXTRACTION_IMAGE, true);
         debugExtractionPanel.create();
         panel.addChild(debugExtractionPanel.getPanel());
      }

      gpuRegionParametersROS2Input = new StoredPropertySetROS2Input(ros2Helper, GPUPlanarRegionExtractionComms.PARAMETERS_OUTPUT, gpuRegionParameters);
      polygonizerParametersROS2Input = new StoredPropertySetROS2Input(ros2Helper,
                                                                      GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS_OUTPUT,
                                                                      polygonizerParameters);
      concaveHullFactoryParametersROS2Input = new StoredPropertySetROS2Input(ros2Helper,
                                                                             GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS_OUTPUT,
                                                                             concaveHullFactoryParameters);
      gpuRegionParametersTuner = new ImGuiStoredPropertySetTuner(gpuRegionParameters.getTitle());
      gpuRegionParametersTuner.create(gpuRegionParameters, () -> gpuRegionParametersChangedByImGuiUser = true);
      setConcaveHullFactoryImGuiWidgetsFromParameters();
      setPolygonizerImGuiWidgetsFromParameters();
   }

   public void update()
   {
      if (ros1Helper != null)
      {
         debugExtractionPanel.updateSubscribers(ros1Helper);
         debugExtractionPanel.update();
      }
   }

   public void renderImGuiWidgets()
   {
      if (ros1Helper != null)
         debugExtractionPanel.renderImGuiWidgets();

      if (ImGui.button("Update parameters from remote"))
      {
         gpuRegionParametersROS2Input.setToAcceptUpdate();
         polygonizerParametersROS2Input.setToAcceptUpdate();
         concaveHullFactoryParametersROS2Input.setToAcceptUpdate();
      }

      if (ImGui.button("Reconnect remote ROS 1 node"))
      {
         ros2Helper.publish(GPUPlanarRegionExtractionComms.RECONNECT_ROS1_NODE);
      }

      ImGui.separator();

      gpuRegionParametersROS2Input.update();

      ImGui.text("GPU Planar Regions Parameters");
      if (gpuRegionParametersROS2Input.getWaitingForUpdate())
      {
         ImGui.text("Waiting for updated values from remote...");
      }
      else
      {
         renderGPUParameterWidgets();
      }

      ImGui.separator();

      if (polygonizerParametersROS2Input.update())
      {
         setPolygonizerImGuiWidgetsFromParameters();
      }

      ImGui.text("Polygonizer Parameters");
      if (polygonizerParametersROS2Input.getWaitingForUpdate())
      {
         ImGui.text("Waiting for initial values from remote...");
      }
      else
      {
         renderPolygonizerParameterWidgets();
      }

      ImGui.separator();

      if (concaveHullFactoryParametersROS2Input.update())
      {
         setConcaveHullFactoryImGuiWidgetsFromParameters();
      }

      ImGui.text("Concave Hull Factory Parameters");
      if (concaveHullFactoryParametersROS2Input.getWaitingForUpdate())
      {
         ImGui.text("Waiting for initial values from remote...");
      }
      else
      {
         renderConcaveHullFactoryParameterWidgets();
      }
   }

   private void renderGPUParameterWidgets()
   {
      gpuRegionParametersTuner.renderImGuiWidgets();

      if (gpuRegionParametersChangedByImGuiUser)
      {
         gpuRegionParametersChangedByImGuiUser = false;
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
      ImGui.sliderFloat("Edge Length Threshold", edgeLengthThresholdSlider.getData(), 0, 0.5f);
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
      concaveHullFactoryParameters.setEdgeLengthThreshold(edgeLengthThresholdSlider.get());
      concaveHullFactoryParameters.setTriangulationTolerance(triangulationToleranceSlider.get());
      concaveHullFactoryParameters.setMaxNumberOfIterations(maxNumberOfIterationsSlider.get());
      concaveHullFactoryParameters.setRemoveAllTrianglesWithTwoBorderEdges(removeAllTrianglesWithTwoBorderEdgesChecked.get());
      concaveHullFactoryParameters.setAllowSplittingConcaveHull(allowSplittingConcaveHullChecked.get());
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
      edgeLengthThresholdSlider.set((float) concaveHullFactoryParameters.getEdgeLengthThreshold());
      triangulationToleranceSlider.set((float) concaveHullFactoryParameters.getTriangulationTolerance());
      maxNumberOfIterationsSlider.set(concaveHullFactoryParameters.getMaxNumberOfIterations());
      removeAllTrianglesWithTwoBorderEdgesChecked.set(concaveHullFactoryParameters.getRemoveAllTrianglesWithTwoBorderEdges());
      allowSplittingConcaveHullChecked.set(concaveHullFactoryParameters.getAllowSplittingConcaveHull());
   }

   public void destroy()
   {
      if (debugExtractionPanel != null)
         debugExtractionPanel.destroy();
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
