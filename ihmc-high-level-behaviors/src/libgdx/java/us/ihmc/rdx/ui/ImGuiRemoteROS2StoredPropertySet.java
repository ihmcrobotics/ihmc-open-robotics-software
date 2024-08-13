package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.communication.property.StoredPropertySetROS2Input;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.tools.property.StoredPropertySetBasics;

public class ImGuiRemoteROS2StoredPropertySet
{
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final StoredPropertySetBasics storedPropertySet;
   private final StoredPropertySetROS2TopicPair topicPair;
   private final StoredPropertySetROS2Input storedPropertySetROS2Input;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXStoredPropertySetTuner imGuiStoredPropertySetTuner;
   private boolean storedPropertySetChangedByImGuiUser = false;
   private static final Color DARK_RED = new Color(0x781d1dff);
   private static final Color YELLOW = new Color(0xa6b51bff);

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
      ros2PublishSubscribeAPI.createPublisher(topicPair.getCommandTopic());

      storedPropertySetROS2Input = new StoredPropertySetROS2Input(ros2PublishSubscribeAPI, topicPair.getStatusTopic(), storedPropertySet);
      imGuiStoredPropertySetTuner = new RDXStoredPropertySetTuner(storedPropertySet.getTitle());
      imGuiStoredPropertySetTuner.create(storedPropertySet, false, () -> storedPropertySetChangedByImGuiUser = true);
   }

   public void setToAcceptUpdate()
   {
      storedPropertySetROS2Input.setToAcceptUpdate();
   }

   public void renderImGuiWidgetsWithUpdateButton()
   {
      ImGui.text("# " + storedPropertySetROS2Input.getNumberOfMessagesReceived() + ": ");
      ImGui.sameLine();
      if (storedPropertySetROS2Input.getUpdateAvailable())
      {
         ImGuiTools.textColored(YELLOW, "[!] Updated parameters are available.");
         ImGui.sameLine();
         if (ImGui.button(labels.get("Accept")))
         {
            storedPropertySetROS2Input.setToAcceptUpdate();
         }
      }
      else if (storedPropertySetROS2Input.getWaitingForUpdate())
      {
         ImGuiTools.textColored(YELLOW, "[!] Waiting for updated values from remote.");
      }
      else if (storedPropertySetROS2Input.getIsExpired())
      {
         ImGuiTools.textColored(DARK_RED, "[!] Parameters have expired.");
      }
      else
      {
         ImGui.text("Parameters are up to date.");

      }
      renderImGuiWidgets();
   }

   public void renderImGuiWidgets()
   {
      storedPropertySetROS2Input.update();

      ImGui.beginDisabled(storedPropertySetROS2Input.getWaitingForUpdate());
      imGuiStoredPropertySetTuner.renderImGuiWidgets();
      publishIfNecessary();
      ImGui.endDisabled();
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

   public RDXPanel createPanel()
   {
      return new RDXPanel(storedPropertySet.getTitle(), this::renderImGuiWidgetsWithUpdateButton);
   }
}
