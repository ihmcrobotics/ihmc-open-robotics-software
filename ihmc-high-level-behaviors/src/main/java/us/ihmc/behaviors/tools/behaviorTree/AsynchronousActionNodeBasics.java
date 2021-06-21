package us.ihmc.behaviors.tools.behaviorTree;

import us.ihmc.log.LogTools;

import static us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

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
public interface AsynchronousActionNodeBasics extends ResettingNodeBasics
{
   @Override
   public default BehaviorTreeNodeStatus tickInternal()
   {
      if (!getHasStarted())
      {
         setHasStarted(true);
         doAction();
         return RUNNING;
      }
      else if (!getIsFinished())
      {
         return RUNNING;
      }
      else
      {
         return getFinishedStatus();
      }
   }

   public default void reset()
   {
      if (getHasStarted() && !getIsFinished())
      {
         LogTools.warn("Task was still running after it wasn't being ticked! {}", getClass().getSimpleName());
         resetInternal();
      }
      setHasStarted(false);
      setIsFinished(false);
   }

   public default void doAction()
   {
      setFinishedStatus(doActionInternal());
      setIsFinished(true);
   }

   public abstract BehaviorTreeNodeStatus doActionInternal();

   public abstract void resetInternal();

   public abstract boolean getHasStarted();

   public abstract void setHasStarted(boolean hasStarted);

   public abstract boolean getIsFinished();

   public abstract void setIsFinished(boolean isFinished);

   public abstract BehaviorTreeNodeStatus getFinishedStatus();

   public abstract void setFinishedStatus(BehaviorTreeNodeStatus finishedStatus);
}
