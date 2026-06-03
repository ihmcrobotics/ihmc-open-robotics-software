package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.Color;
import ihmc_common_msgs.PrimitiveDataVectorMessage;
import imgui.ImGui;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.communication.property.StoredPropertySetROS2Input;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.tools.property.StoredPropertySetBasics;

public class ImGuiRemoteROS2StoredPropertySet
{
   private final StoredPropertySetBasics storedPropertySet;
   private final StoredPropertySetROS2Input storedPropertySetROS2Input;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXStoredPropertySetTuner imGuiStoredPropertySetTuner;
   private boolean storedPropertySetChangedByImGuiUser = false;
   private static final Color DARK_RED = new Color(0x781d1dff);
   private static final Color YELLOW = new Color(0xa6b51bff);
   private final ROS2Publisher<PrimitiveDataVectorMessage> publisher;

   public ImGuiRemoteROS2StoredPropertySet(ROS2Node ros2Node,
                                           StoredPropertySetBasics storedPropertySet,
                                           StoredPropertySetROS2TopicPair topicPair)
   {
      this.storedPropertySet = storedPropertySet;
      publisher = ros2Node.createPublisher(topicPair.getCommandTopic());

      storedPropertySetROS2Input = new StoredPropertySetROS2Input(ros2Node, topicPair.getStatusTopic(), storedPropertySet);
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
         publisher.publish(StoredPropertySetMessageTools.newMessage(storedPropertySet));
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
