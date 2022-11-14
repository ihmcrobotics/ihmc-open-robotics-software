package us.ihmc.rdx.ui;

import imgui.ImGui;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.communication.property.StoredPropertySetROS2Input;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.property.StoredPropertySetBasics;

public class ImGuiRemoteROS2StoredPropertySet
{
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final StoredPropertySetBasics storedPropertySet;
   private final StoredPropertySetROS2TopicPair topicPair;
   private final StoredPropertySetROS2Input storedPropertySetROS2Input;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiStoredPropertySetTuner imGuiStoredPropertySetTuner;
   private boolean storedPropertySetChangedByImGuiUser = false;

   public ImGuiRemoteROS2StoredPropertySet(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                           StoredPropertySetBasics storedPropertySet,
                                           String moduleTopicName)
   {
      this(ros2PublishSubscribeAPI,
           storedPropertySet,
           new StoredPropertySetROS2TopicPair(moduleTopicName, storedPropertySet));
   }

   public ImGuiRemoteROS2StoredPropertySet(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                           StoredPropertySetBasics storedPropertySet,
                                           StoredPropertySetROS2TopicPair topicPair)
   {
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;
      this.storedPropertySet = storedPropertySet;
      this.topicPair = topicPair;

      storedPropertySetROS2Input = new StoredPropertySetROS2Input(ros2PublishSubscribeAPI, topicPair.getStatusTopic(), storedPropertySet);
      imGuiStoredPropertySetTuner = new ImGuiStoredPropertySetTuner(storedPropertySet.getTitle());
      imGuiStoredPropertySetTuner.create(storedPropertySet, false, () -> storedPropertySetChangedByImGuiUser = true);
   }

   public void setToAcceptUpdate()
   {
      storedPropertySetROS2Input.setToAcceptUpdate();
   }

   public void renderImGuiWidgetsWithUpdateButton()
   {
      if (ImGui.button(labels.get("Update parameters from remote")))
      {
         storedPropertySetROS2Input.setToAcceptUpdate();
      }
      renderImGuiWidgets();
   }

   public void renderImGuiWidgets()
   {
      storedPropertySetROS2Input.update();

      if (storedPropertySetROS2Input.getWaitingForUpdate())
      {
         ImGui.text(storedPropertySet.getTitle());
         ImGui.text("Waiting for updated values from remote...");
      }
      else
      {
         imGuiStoredPropertySetTuner.renderImGuiWidgets();
         publishIfNecessary();
      }
   }

   private void publishIfNecessary()
   {
      if (storedPropertySetChangedByImGuiUser)
      {
         storedPropertySetChangedByImGuiUser = false;
         ros2PublishSubscribeAPI.publish(topicPair.getCommandTopic(), StoredPropertySetMessageTools.newMessage(storedPropertySet));
      }
   }

   public void setPropertyChanged()
   {
      storedPropertySetChangedByImGuiUser = true;
      publishIfNecessary();
   }

   public StoredPropertySetBasics getStoredPropertySet()
   {
      return storedPropertySet;
   }

   public ImGuiPanel createPanel()
   {
      return new ImGuiPanel(storedPropertySet.getTitle(), this::renderImGuiWidgetsWithUpdateButton);
   }
}
