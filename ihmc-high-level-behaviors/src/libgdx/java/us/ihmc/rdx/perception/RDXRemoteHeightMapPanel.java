package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.flag.ImGuiButtonFlags;
import imgui.flag.ImGuiMouseButton;
import perception_msgs.msg.dds.HeightMapStateRequestMessage;
import us.ihmc.communication.ROS2Tools;
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

   private final ROS2Helper ros2Helper;
   private final HeightMapStateRequestMessage pauseMessage = new HeightMapStateRequestMessage();
   private final HeightMapStateRequestMessage resumeMessage = new HeightMapStateRequestMessage();
   private final HeightMapStateRequestMessage clearMessage = new HeightMapStateRequestMessage();

   public RDXRemoteHeightMapPanel(ROS2Helper ros2Helper)
   {
      this.ros2Helper = ros2Helper;
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      remotePropertySets.registerRemotePropertySet(heightMapParameters, HeightMapAPI.PARAMETERS);
      remotePropertySets.registerRemotePropertySet(heightMapFilterParameters, HeightMapAPI.FILTER_PARAMETERS);

      pauseMessage.setRequestPause(true);
      resumeMessage.setRequestResume(true);
      clearMessage.setRequestClear(true);
   }

   public void update()
   {
   }

   public void renderImGuiWidgets()
   {
      remotePropertySets.renderImGuiWidgets();

      if (ImGui.button("Pause"))
         ros2Helper.publish(ROS2Tools.HEIGHT_MAP_STATE_REQUEST, pauseMessage);
      if (ImGui.button("Resume"))
         ros2Helper.publish(ROS2Tools.HEIGHT_MAP_STATE_REQUEST, resumeMessage);
      if (ImGui.button("Clear"))
         ros2Helper.publish(ROS2Tools.HEIGHT_MAP_STATE_REQUEST, clearMessage);
   }

   public void destroy()
   {
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
