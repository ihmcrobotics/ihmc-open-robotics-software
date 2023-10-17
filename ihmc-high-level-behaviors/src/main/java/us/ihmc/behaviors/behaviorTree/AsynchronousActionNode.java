package us.ihmc.behaviors.behaviorTree;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

/**
 * A Technique for Parallelism with Behavior Trees
 *
 * A node starts on its task in a new Thread the first time it is ticked.
 * The node returns running until the task is complete.
 * When the task is complete, the node returns success or failure until it is reset.
 * A node runs its task while it is being constantly ticked.
 * An additional clock() method is used by a node to determine that the tree has gone elsewhere and that it should abort its task and reset.
 *
 */
public abstract class AsynchronousActionNode extends ResettingNode
{
   private boolean hasStarted = false;
   private boolean isFinished = false;
   private BehaviorTreeNodeStatus finishedStatus;
   private Thread thread;

   public AsynchronousActionNode()
   {

   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      if (!getHasStarted())
      {
         setHasStarted(true);
         doAction();
         return BehaviorTreeNodeStatus.RUNNING;
      }
      else if (!getIsFinished())
      {
         return BehaviorTreeNodeStatus.RUNNING;
      }
      else
      {
         return getFinishedStatus();
      }
   }

   public void reset()
   {
      if (getHasStarted() && !getIsFinished())
      {
         LogTools.warn("Task was still running after it wasn't being ticked! {}", getClass().getSimpleName());
         resetInternal();
      }
      setHasStarted(false);
      setIsFinished(false);
   }

   public void doAction()
   {
      setFinishedStatus(doActionInternal());
      setIsFinished(true);
   }

   public BehaviorTreeNodeStatus doActionInternal()
   {
      thread = ThreadTools.startAThread(this::doAction, getClass().getSimpleName());
      return BehaviorTreeNodeStatus.RUNNING;
   }

   public abstract void resetInternal();

   public boolean getHasStarted()
   {
      return hasStarted;
   }

   public void setHasStarted(boolean hasStarted)
   {
      this.hasStarted = hasStarted;
   }

   public boolean getIsFinished()
   {
      return isFinished;
   }

   public void setIsFinished(boolean isFinished)
   {
      this.isFinished = isFinished;
   }

   public BehaviorTreeNodeStatus getFinishedStatus()
   {
      return finishedStatus;
   }

   public void setFinishedStatus(BehaviorTreeNodeStatus finishedStatus)
   {
      this.finishedStatus = finishedStatus;
   }
}
