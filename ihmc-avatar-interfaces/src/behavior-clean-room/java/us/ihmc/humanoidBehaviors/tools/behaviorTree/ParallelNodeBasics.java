package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Timer;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.RUNNING;
import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.SUCCESS;

public abstract class ParallelNodeBasics implements BehaviorTreeNode
{
   private final double expectedTickPeriod;
   private final Timer deactivationTimer = new Timer();

   private boolean hasStarted = false;
   private boolean isFinished = false;

   public ParallelNodeBasics(double expectedTickPeriod)
   {
      this.expectedTickPeriod = expectedTickPeriod;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (deactivationTimer.isExpired(expectedTickPeriod * 1.5))
      {
         if (hasStarted && !isFinished)
            LogTools.warn("Task was still running after it wasn't being ticked!");
         hasStarted = false;
         isFinished = false;
      }

      deactivationTimer.reset();

      if (!hasStarted)
      {
         hasStarted = true;
         ThreadTools.startAThread(this::doActionInternal, getClass().getSimpleName());
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

   public void reset()
   {
      hasStarted = false;
      isFinished = false;
   }

   private void doActionInternal()
   {
      doAction();
      isFinished = true;
   }

   public abstract void doAction();
}
