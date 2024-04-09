package us.ihmc.communication.ros2;
import std_msgs.msg.dds.Empty;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.Arrays;

public class ROS2DemandGraphNode
{
   private final ArrayList<ROS2DemandGraphNode> dependents = new ArrayList<>();
   private final ROS2HeartbeatMonitor nodeHeartbeatMonitor;

   public ROS2DemandGraphNode(ROS2PublishSubscribeAPI ros2, ROS2Topic<Empty> heartbeatTopic)
   {
      nodeHeartbeatMonitor = new ROS2HeartbeatMonitor(ros2, heartbeatTopic);
   }

   public void addDependents(ROS2DemandGraphNode... dependents)
   {
      this.dependents.addAll(Arrays.stream(dependents).toList());
   }

   public boolean isDemanded()
   {
      return true;

//      if (nodeHeartbeatMonitor.isAlive())
//         return true;
//
//      for (ROS2DemandGraphNode dependent : dependents)
//      {
//         if (dependent.isDemanded())
//            return true;
//      }
//
//      return false;
   }

   public void destroy()
   {
      for (ROS2DemandGraphNode dependent : dependents)
         dependent.destroy();

      nodeHeartbeatMonitor.destroy();
   }
}
