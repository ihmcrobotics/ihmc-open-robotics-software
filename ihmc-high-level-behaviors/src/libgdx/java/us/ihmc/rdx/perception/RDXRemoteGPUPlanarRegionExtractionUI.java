package us.ihmc.rdx.perception;

import imgui.ImGui;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionComms;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
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
   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;

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

      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      remotePropertySets.registerRemotePropertySet(gpuRegionParameters, GPUPlanarRegionExtractionComms.PARAMETERS);
      remotePropertySets.registerRemotePropertySet(polygonizerParameters, GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS);
      remotePropertySets.registerRemotePropertySet(concaveHullFactoryParameters, GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS);
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

      if (ImGui.button(labels.get("Reconnect remote ROS 1 node")))
      {
         ros2Helper.publish(GPUPlanarRegionExtractionComms.RECONNECT_ROS1_NODE);
      }

      remotePropertySets.renderImGuiWidgets();
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
