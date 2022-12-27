package us.ihmc.rdx.perception;

import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionComms;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class RDXRemoteHeightMapUI
{
   private final HeightMapParameters gpuRegionParameters = new HeightMapParameters();
   private final ImGuiPanel panel = new ImGuiPanel("CPU Height Map", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;

   public RDXRemoteHeightMapUI(ROS2Helper ros2Helper)
   {
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      // FIXME fix the comms
      remotePropertySets.registerRemotePropertySet(gpuRegionParameters, GPUPlanarRegionExtractionComms.PARAMETERS);
   }

   public void update()
   {
   }

   public void renderImGuiWidgets()
   {
      remotePropertySets.renderImGuiWidgets();
   }

   public void destroy()
   {
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
