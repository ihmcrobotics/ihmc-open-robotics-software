package us.ihmc.communication.ros2;
import std_msgs.msg.dds.Empty;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class ROS2DemandGraphNode
{
   private final ArrayList<ROS2DemandGraphNode> dependents = new ArrayList<>();
   private final ROS2HeartbeatMonitor nodeHeartbeatMonitor;
   private final Set<Consumer<Boolean>> demandChangedCallbacks = new HashSet<>();
   private final AtomicBoolean wasDemanded = new AtomicBoolean(false);

   public ROS2DemandGraphNode(ROS2PublishSubscribeAPI ros2, ROS2Topic<Empty> heartbeatTopic)
   {
      nodeHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, heartbeatTopic);
      nodeHeartbeatMonitor.setAlivenessChangedCallback(this::checkIfDemandChanged);
   }

   public void addDependents(ROS2DemandGraphNode... dependentsToAdd)
   {
      dependents.addAll(Arrays.stream(dependentsToAdd).toList());
      for (ROS2DemandGraphNode dependent : dependentsToAdd)
         dependent.addDemandChangedCallback(this::checkIfDemandChanged);
   }

   public void addDemandChangedCallback(Consumer<Boolean> demandChangedCallback)
   {
      demandChangedCallbacks.add(demandChangedCallback);
   }

   private void checkIfDemandChanged(boolean ignored)
   {
      boolean isDemanded = isDemanded();
      // if demanded status changed, update wasDemanded to current status and trigger callbacks
      if (wasDemanded.compareAndSet(!isDemanded, isDemanded))
         for (Consumer<Boolean> demandChangedCallback : demandChangedCallbacks)
            demandChangedCallback.accept(isDemanded);
   }

   public boolean isDemanded()
   {
      if (nodeHeartbeatMonitor.isAlive())
         return true;

      for (ROS2DemandGraphNode dependent : dependents)
      {
         if (dependent.isDemanded())
            return true;
      }

      return false;
   }

   public void destroy()
   {
      for (ROS2DemandGraphNode dependent : dependents)
         dependent.destroy();

      nodeHeartbeatMonitor.destroy();
   }
}
