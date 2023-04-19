package us.ihmc.rdx.perception;

import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;

public class RDXActiveMappingRemoteUI
{
   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;
   private final ImGuiPanel panel = new ImGuiPanel("Perception Panel", this::renderImGuiWidgets);

   private final PerceptionConfigurationParameters perceptionConfigurationParameters = new PerceptionConfigurationParameters();

   public RDXActiveMappingRemoteUI(ROS2Helper ros2Helper)
   {
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);

      remotePropertySets.registerRemotePropertySet(perceptionConfigurationParameters, PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS);
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
