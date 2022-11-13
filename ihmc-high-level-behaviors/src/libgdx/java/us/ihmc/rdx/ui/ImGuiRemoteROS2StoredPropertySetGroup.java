package us.ihmc.rdx.ui;

import imgui.ImGui;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.ArrayList;

public class ImGuiRemoteROS2StoredPropertySetGroup
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final ArrayList<ImGuiRemoteROS2StoredPropertySet> remotePropertySets = new ArrayList<>();

   public ImGuiRemoteROS2StoredPropertySetGroup(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;
   }

   public void registerRemotePropertySet(StoredPropertySetBasics storedPropertySet, StoredPropertySetROS2TopicPair topicPair)
   {
      remotePropertySets.add(new ImGuiRemoteROS2StoredPropertySet(ros2PublishSubscribeAPI, storedPropertySet, topicPair));
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button(labels.get("Update parameters from remote")))
      {
         for (ImGuiRemoteROS2StoredPropertySet remotePropertySet : remotePropertySets)
         {
            remotePropertySet.setToAcceptUpdate();
         }
      }

      for (ImGuiRemoteROS2StoredPropertySet remotePropertySet : remotePropertySets)
      {
         ImGui.separator();
         remotePropertySet.renderImGuiWidgets();
      }
   }
}
