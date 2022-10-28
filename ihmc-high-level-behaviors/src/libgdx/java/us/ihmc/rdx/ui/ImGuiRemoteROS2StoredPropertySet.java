package us.ihmc.rdx.ui;

import ihmc_common_msgs.msg.dds.StoredPropertySetMessage;
import imgui.ImGui;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.communication.property.StoredPropertySetROS2Input;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.property.StoredPropertySetBasics;

public class ImGuiRemoteROS2StoredPropertySet
{
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final StoredPropertySetBasics storedPropertySet;
   private final ROS2Topic<StoredPropertySetMessage> outputTopic;
   private final StoredPropertySetROS2Input storedPropertySetROS2Input;
   private final ImGuiStoredPropertySetTuner imGuiStoredPropertySetTuner;
   private boolean storedPropertySetChangedByImGuiUser = false;

   public ImGuiRemoteROS2StoredPropertySet(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                           StoredPropertySetBasics storedPropertySet,
                                           ROS2Topic<StoredPropertySetMessage> inputTopic,
                                           ROS2Topic<StoredPropertySetMessage> outputTopic)
   {
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;
      this.storedPropertySet = storedPropertySet;
      this.outputTopic = outputTopic;
      storedPropertySetROS2Input = new StoredPropertySetROS2Input(ros2PublishSubscribeAPI, inputTopic, storedPropertySet);
      imGuiStoredPropertySetTuner = new ImGuiStoredPropertySetTuner(storedPropertySet.getTitle());
      imGuiStoredPropertySetTuner.create(storedPropertySet, () -> storedPropertySetChangedByImGuiUser = true);
   }

   public void setToAcceptUpdate()
   {
      storedPropertySetROS2Input.setToAcceptUpdate();
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

         if (storedPropertySetChangedByImGuiUser)
         {
            storedPropertySetChangedByImGuiUser = false;
            ros2PublishSubscribeAPI.publish(outputTopic, StoredPropertySetMessageTools.newMessage(storedPropertySet));
         }
      }
   }
}
