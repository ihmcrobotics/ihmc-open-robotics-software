package us.ihmc.rdx.perception;

import imgui.ImGui;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionComms;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySet;
import us.ihmc.rdx.ui.graphics.live.RDXROS1VideoVisualizer;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.utilities.ros.ROS1Helper;

public class RDXRemoteGPUPlanarRegionExtractionUI
{
   private final ROS1Helper ros1Helper;
   private final ROS2Helper ros2Helper;
   private final GPUPlanarRegionExtractionParameters gpuRegionParameters = new GPUPlanarRegionExtractionParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private RDXROS1VideoVisualizer debugExtractionPanel;
   private final ImGuiPanel panel = new ImGuiPanel("GPU Planar Regions", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiRemoteROS2StoredPropertySet remoteGPURegionParameters;
   private final ImGuiRemoteROS2StoredPropertySet remotePolygonizerParameters;
   private final ImGuiRemoteROS2StoredPropertySet remoteConcaveHullFactoryParameters;

   public RDXRemoteGPUPlanarRegionExtractionUI(ROS1Helper ros1Helper, ROS2Helper ros2Helper)
   {
      this.ros1Helper = ros1Helper;
      this.ros2Helper = ros2Helper;

      if (ros1Helper != null)
      {
         debugExtractionPanel = new RDXROS1VideoVisualizer("GPU Planar Regions Debug Image", GPUPlanarRegionExtractionComms.DEBUG_EXTRACTION_IMAGE, true);
         debugExtractionPanel.create();
         panel.addChild(debugExtractionPanel.getPanel());
      }

      remoteGPURegionParameters = new ImGuiRemoteROS2StoredPropertySet(ros2Helper,
                                                                       gpuRegionParameters,
                                                                       GPUPlanarRegionExtractionComms.PARAMETERS_OUTPUT,
                                                                       GPUPlanarRegionExtractionComms.PARAMETERS_INPUT);
      remotePolygonizerParameters = new ImGuiRemoteROS2StoredPropertySet(ros2Helper,
                                                                         polygonizerParameters,
                                                                         GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS_OUTPUT,
                                                                         GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS_INPUT);
      remoteConcaveHullFactoryParameters = new ImGuiRemoteROS2StoredPropertySet(ros2Helper,
                                                                                concaveHullFactoryParameters,
                                                                                GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS_OUTPUT,
                                                                                GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS_INPUT);
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

      if (ImGui.button(labels.get("Update parameters from remote")))
      {
         remoteGPURegionParameters.setToAcceptUpdate();
         remotePolygonizerParameters.setToAcceptUpdate();
         remoteConcaveHullFactoryParameters.setToAcceptUpdate();
      }

      if (ImGui.button(labels.get("Reconnect remote ROS 1 node")))
      {
         ros2Helper.publish(GPUPlanarRegionExtractionComms.RECONNECT_ROS1_NODE);
      }

      ImGui.separator();
      remoteGPURegionParameters.renderImGuiWidgets();
      ImGui.separator();
      remotePolygonizerParameters.renderImGuiWidgets();
      ImGui.separator();
      remoteConcaveHullFactoryParameters.renderImGuiWidgets();
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
