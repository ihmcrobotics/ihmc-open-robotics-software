package us.ihmc.rdx.ui.algorithmUI;

import imgui.type.ImBoolean;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySet;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.property.StoredPropertySetBasics;

import javax.annotation.Nullable;

public class RDXTunableAlgorithm extends RDXPanel
{
   private final ImBoolean active;
   private final String title;

   private final ROS2PublishSubscribeAPI ros2;
   private final StoredPropertySetBasics algorithmPropertySet;
   private final StoredPropertySetROS2TopicPair propertySetTopicPair;
   private ImGuiRemoteROS2StoredPropertySet imGuiPropertySet;

   @Nullable
   private ROS2Heartbeat heartbeat;

   public RDXTunableAlgorithm(StoredPropertySetBasics algorithmPropertySet, StoredPropertySetROS2TopicPair propertySetTopicPair, ROS2PublishSubscribeAPI ros2)
   {
      this(algorithmPropertySet.getTitle(), algorithmPropertySet, propertySetTopicPair, ros2, false);
   }

   public RDXTunableAlgorithm(String title,
                              StoredPropertySetBasics algorithmPropertySet,
                              StoredPropertySetROS2TopicPair propertySetTopicPair,
                              ROS2PublishSubscribeAPI ros2,
                              boolean activeByDefault)
   {
      super(title);

      this.title = title;
      this.algorithmPropertySet = algorithmPropertySet;
      this.propertySetTopicPair = propertySetTopicPair;
      this.ros2 = ros2;

      active = new ImBoolean(activeByDefault);

      setRenderMethod(this::renderImGuiWidgets);
   }

   public void create()
   {
      if (!isCreated())
         imGuiPropertySet = new ImGuiRemoteROS2StoredPropertySet(ros2, algorithmPropertySet, propertySetTopicPair);
   }

   public void create(RDXStoredPropertySetTuner tunerUI)
   {
      if (!isCreated())
         imGuiPropertySet = new ImGuiRemoteROS2StoredPropertySet(ros2, algorithmPropertySet, propertySetTopicPair, tunerUI);
   }

   public void createRequestHeartbeat(ROS2Topic<Empty> heartbeatTopic)
   {
      heartbeat = new ROS2Heartbeat(ros2, heartbeatTopic);
      heartbeat.setAlive(isActive());
   }

   public void renderImGuiWidgets()
   {
      if (imGuiPropertySet != null)
         imGuiPropertySet.renderImGuiWidgetsWithUpdateButton();
   }

   public void updateHeartbeat()
   {
      if (heartbeat != null)
         heartbeat.setAlive(isActive());
   }

   public boolean hasHeartbeat()
   {
      return heartbeat != null;
   }

   public boolean isActive()
   {
      return active.get();
   }

   public ImBoolean getActive()
   {
      return active;
   }

   public boolean isCreated()
   {
      return imGuiPropertySet != null;
   }

   public String getTitle()
   {
      return title;
   }
}
