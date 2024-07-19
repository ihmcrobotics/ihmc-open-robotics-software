package us.ihmc.rdx.ui.algorithmUI;

import imgui.ImGui;
import imgui.type.ImBoolean;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.property.StoredPropertySetBasics;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class RDXActivatableAlgorithm extends RDXTunableAlgorithm
{
   private final ImBoolean active;
   private final List<Consumer<Boolean>> activenessChangeCallbacks = new ArrayList<>();
   @Nullable
   private ROS2Heartbeat heartbeat = null;

   public RDXActivatableAlgorithm(StoredPropertySetBasics algorithmPropertySet,
                                  StoredPropertySetROS2TopicPair propertySetTopicPair,
                                  ROS2PublishSubscribeAPI ros2)
   {
      this(algorithmPropertySet.getTitle(), algorithmPropertySet, propertySetTopicPair, ros2, false);
   }

   public RDXActivatableAlgorithm(String title,
                                  StoredPropertySetBasics algorithmPropertySet,
                                  StoredPropertySetROS2TopicPair propertySetTopicPair,
                                  ROS2PublishSubscribeAPI ros2)
   {
      this(title, algorithmPropertySet, propertySetTopicPair, ros2, false);
   }

   public RDXActivatableAlgorithm(String title,
                                  StoredPropertySetBasics algorithmPropertySet,
                                  StoredPropertySetROS2TopicPair propertySetTopicPair,
                                  ROS2PublishSubscribeAPI ros2,
                                  boolean activeByDefault)
   {
      super(title, algorithmPropertySet, propertySetTopicPair, ros2);

      active = new ImBoolean(activeByDefault);
   }

   @Override
   public boolean renderMenuEntry()
   {
      if (ImGui.checkbox(labels.get("##active"), isActive()))
      {
         setActive(!isActive());
      }
      ImGui.sameLine();

      return super.renderMenuEntry();
   }

   public void createRequestHeartbeat(ROS2Topic<Empty> heartbeatTopic)
   {
      heartbeat = new ROS2Heartbeat(ros2, heartbeatTopic);
      heartbeat.setAlive(isActive());
      addActivenessChangeCallback(isActive -> heartbeat.setAlive(isActive));
   }

   public void addActivenessChangeCallback(Consumer<Boolean> activenessChangeCallback)
   {
      activenessChangeCallbacks.add(activenessChangeCallback);
   }

   public void setActive(boolean active)
   {
      boolean wasActive = this.active.get();
      this.active.set(active);
      if (wasActive != isActive())
      {
         for (Consumer<Boolean> callback : activenessChangeCallbacks)
            callback.accept(isActive());
      }
   }

   public boolean isActive()
   {
      return active.get();
   }
}
