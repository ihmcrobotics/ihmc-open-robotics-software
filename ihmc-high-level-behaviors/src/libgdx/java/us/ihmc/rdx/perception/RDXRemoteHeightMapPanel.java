package us.ihmc.rdx.perception;

import us.ihmc.ihmcPerception.heightMap.HeightMapAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.sensorProcessing.heightMap.HeightMapFilterParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class RDXRemoteHeightMapPanel
{
   private final HeightMapParameters heightMapParameters = new HeightMapParameters();
   private final HeightMapFilterParameters heightMapFilterParameters = new HeightMapFilterParameters();
   private final ImGuiPanel panel = new ImGuiPanel("CPU Height Map", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;

   public RDXRemoteHeightMapPanel(ROS2Helper ros2Helper)
   {
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      remotePropertySets.registerRemotePropertySet(heightMapParameters, HeightMapAPI.PARAMETERS);
      remotePropertySets.registerRemotePropertySet(heightMapFilterParameters, HeightMapAPI.FILTER_PARAMETERS);
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
