package us.ihmc.behaviors.tools.behaviorTree;

import us.ihmc.commons.thread.ThreadTools;

public abstract class AsynchronousActionNode extends BehaviorTreeControlFlowNode implements AsynchronousActionNodeBasics
{
   private boolean hasStarted = false;
   private boolean isFinished = false;

   private boolean lastWasClock = false;
   private BehaviorTreeNodeStatus finishedStatus;
   private Thread thread;

   public AsynchronousActionNode()
   {
      setType(AsynchronousActionNode.class);
   }

   @Override
   public void startAction()
   {
      thread = ThreadTools.startAThread(this::doAction, getClass().getSimpleName());
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
   public boolean getLastWasClock()
   {
      return lastWasClock;
   }

   @Override
   public void setLastWasClock(boolean lastWasClock)
   {
      this.lastWasClock = lastWasClock;
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
