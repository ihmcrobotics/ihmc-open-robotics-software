package us.ihmc.rdx.ui;

import imgui.ImGui;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.ArrayList;

public class ImGuiRemoteROS2StoredPropertySetGroup
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2Node ros2Node;
   private final ArrayList<ImGuiRemoteROS2StoredPropertySet> remotePropertySets = new ArrayList<>();

   public ImGuiRemoteROS2StoredPropertySetGroup(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public void registerRemotePropertySet(StoredPropertySetBasics storedPropertySet, StoredPropertySetROS2TopicPair topicPair)
   {
      remotePropertySets.add(new ImGuiRemoteROS2StoredPropertySet(ros2Node, storedPropertySet, topicPair));
   }

   public void renderImGuiWidgets()
   {
      for (int i = 0; i < remotePropertySets.size(); i++)
      {
         ImGuiRemoteROS2StoredPropertySet remotePropertySet = remotePropertySets.get(i);

         if (ImGui.collapsingHeader(remotePropertySet.getStoredPropertySet().getTitle()))
         {
            ImGui.indent();
            remotePropertySet.renderImGuiWidgets();
            ImGui.unindent();
         }
      }
   }

   public void setPropertyChanged()
   {
      for (int i = 0; i < remotePropertySets.size(); i++)
      {
         ImGuiRemoteROS2StoredPropertySet remotePropertySet = remotePropertySets.get(i);
         remotePropertySet.setPropertyChanged();
      }
   }
}
