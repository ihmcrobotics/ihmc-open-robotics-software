package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.RUNNING;
import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.SUCCESS;

public abstract class ParallelNodeBasics implements BehaviorTreeNode
{
   private boolean hasStarted = false;
   private boolean isFinished = false;

   private boolean lastWasClock = false;
   private boolean resetRequired = false;

   public ParallelNodeBasics()
   {
   }

   @Override
   public void clock()
   {
      if (lastWasClock)
      {
         resetRequired = true;
      }

      lastWasClock = true;
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (resetRequired)
      {
         resetRequired = false;
         if (hasStarted && !isFinished)
            LogTools.warn("Task was still running after it wasn't being ticked! {}", getClass().getSimpleName());

         reset();
      }

      lastWasClock = false;

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
