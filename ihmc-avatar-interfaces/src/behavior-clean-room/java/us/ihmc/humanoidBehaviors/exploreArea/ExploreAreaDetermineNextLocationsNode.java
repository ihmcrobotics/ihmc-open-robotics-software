package us.ihmc.humanoidBehaviors.exploreArea;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNode;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.tools.Timer;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.RUNNING;
import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.SUCCESS;

public class ExploreAreaDetermineNextLocationsNode implements BehaviorTreeNode
{
   private final double expectedTickPeriod;
   private final Timer deactivationTimer = new Timer();
   boolean hasStarted = false;
   boolean isFinished = false;

   public ExploreAreaDetermineNextLocationsNode(double expectedTickPeriod)
   {
      this.expectedTickPeriod = expectedTickPeriod;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (deactivationTimer.isExpired(expectedTickPeriod * 1.5))
      {
         hasStarted = false;
         isFinished = false;
      }

      deactivationTimer.reset();

      if (!hasStarted)
      {
         hasStarted = true;
         ThreadTools.startAThread(this::runCompute, "DetermineNextLocations");
         return RUNNING;
      }
      else if (!isFinished)
      {
         return RUNNING;
      }
      else
      {
         return SUCCESS;
      }
   }

   private void runCompute()
   {
      

      isFinished = true;
   }
}
