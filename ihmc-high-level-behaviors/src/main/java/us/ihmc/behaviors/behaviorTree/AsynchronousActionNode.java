package us.ihmc.behaviors.behaviorTree;

import us.ihmc.commons.thread.ThreadTools;

public abstract class AsynchronousActionNode extends ResettingNode implements AsynchronousActionNodeBasics
{
   private boolean hasStarted = false;
   private boolean isFinished = false;
   private BehaviorTreeNodeStatus finishedStatus;
   private Thread thread;

   public AsynchronousActionNode()
   {
      setType(AsynchronousActionNode.class);
   }

   @Override
   public BehaviorTreeNodeStatus doActionInternal()
   {
      thread = ThreadTools.startAThread(this::doAction, getClass().getSimpleName());
      return BehaviorTreeNodeStatus.RUNNING;
   }

   @Override
   public boolean getHasStarted()
   {
      return hasStarted;
   }

   @Override
   public void setHasStarted(boolean hasStarted)
   {
      this.hasStarted = hasStarted;
   }

   @Override
   public boolean getIsFinished()
   {
      return isFinished;
   }

   @Override
   public void setIsFinished(boolean isFinished)
   {
      this.isFinished = isFinished;
   }

   @Override
   public BehaviorTreeNodeStatus getFinishedStatus()
   {
      return finishedStatus;
   }

   @Override
   public void setFinishedStatus(BehaviorTreeNodeStatus finishedStatus)
   {
      this.finishedStatus = finishedStatus;
   }
}
