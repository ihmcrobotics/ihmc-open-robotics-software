package us.ihmc.communication.ros2;
import std_msgs.msg.dds.Empty;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.Arrays;

public class ROS2HeartbeatDependencyNode
{
   private final ArrayList<ROS2HeartbeatDependencyNode> dependants = new ArrayList<>();
   private final ROS2HeartbeatMonitor nodeHeartbeatMonitor;

   public ROS2HeartbeatDependencyNode(ROS2PublishSubscribeAPI ros2, ROS2Topic<Empty> heartbeatTopic)
   {
      nodeHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, heartbeatTopic);
   }

   public void addDependants(ROS2HeartbeatDependencyNode... dependants)
   {
      this.dependants.addAll(Arrays.stream(dependants).toList());
   }

   public boolean checkIfDesired()
   {
      if (nodeHeartbeatMonitor.isAlive())
         return true;

      for (ROS2HeartbeatDependencyNode dependant : dependants)
      {
         if (dependant.checkIfDesired())
            return true;
      }

      return false;
   }

   public void destroy()
   {
      for (ROS2HeartbeatDependencyNode dependant : dependants)
         dependant.destroy();

      nodeHeartbeatMonitor.destroy();
   }
}
