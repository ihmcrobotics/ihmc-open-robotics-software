package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.communication.ROS2Callback;
import us.ihmc.ros2.Ros2Node;

import java.util.ArrayList;
import java.util.List;

public class ManagedROS2Node
{
   private final List<ROS2Callback> ros2Callbacks = new ArrayList<>();
   private final Ros2Node ros2Node;

   public ManagedROS2Node(Ros2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public void setEnabled(boolean enabled)
   {
      for (ROS2Callback ros2Callback : ros2Callbacks)
      {
         ros2Callback.setEnabled(enabled);
      }
   }
}
