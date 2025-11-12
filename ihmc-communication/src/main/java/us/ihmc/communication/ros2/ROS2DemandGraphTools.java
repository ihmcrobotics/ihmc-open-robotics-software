package us.ihmc.communication.ros2;

import us.ihmc.commons.thread.RepeatingTaskThread;

import java.util.Collection;

public class ROS2DemandGraphTools
{
   public static boolean anyDemanded(Collection<ROS2DemandGraphNode> demandNodes)
   {
      for (ROS2DemandGraphNode demandNode : demandNodes)
      {
         if (demandNode.isDemanded())
            return true;
      }

      return false;
   }

   public static void runWhileDemanded(RepeatingTaskThread threadToRun, ROS2DemandGraphNode demandNode)
   {
      if (!threadToRun.isAlive())
         threadToRun.start();

      if (demandNode.isDemanded())
         threadToRun.startRepeating();

      demandNode.addDemandChangedCallback(threadToRun::setRepeating);
   }

   public static void runWhileAnyDemanded(RepeatingTaskThread threadToRun, Collection<ROS2DemandGraphNode> demandNodes)
   {
      if (!threadToRun.isAlive())
         threadToRun.start();

      for (ROS2DemandGraphNode demandNode : demandNodes)
      {
         if (demandNode.isDemanded())
            threadToRun.startRepeating();

         demandNode.addDemandChangedCallback(demanded -> threadToRun.setRepeating(anyDemanded(demandNodes)));
      }
   }
}
