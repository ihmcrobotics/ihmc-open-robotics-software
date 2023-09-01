package us.ihmc.behaviors.tools.behaviorTree;

import java.time.Instant;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public interface BehaviorTreeNodeBasics
{
   public default double evaluateUtility()
   {
      return 1.0;
   }

   /**
    * A method that can be called on every node in the tree every time the root gets ticked
    * in order for parallel nodes to figure out when they are no longer being selected.
    */
   public default void clock()
   {

   }

   public default BehaviorTreeNodeStatus tick()
   {
      setPreviousStatus(tickInternal());
      setLastTickInstant(Instant.now());
      return getPreviousStatus();
   }

   public abstract BehaviorTreeNodeStatus tickInternal();

   /**
    * @return The node's status from the last time it was ticked.
    *         This will be null if the node hasn't been ticked yet.
    */
   public abstract BehaviorTreeNodeStatus getPreviousStatus();

   public abstract void setPreviousStatus(BehaviorTreeNodeStatus status);

   public default boolean hasBeenTicked()
   {
      return getLastTickInstant() != null;
   }

   /**
    * @return The Instant at which this node was last ticked.
    *         This will be null if the node has never been ticked.
    */
   public abstract Instant getLastTickInstant();

   public abstract void setLastTickInstant(Instant lastTickInstant);

   public abstract String getName();

   public abstract void setName(String name);

   public abstract Class<?> getType();

   public abstract void setType(Class<?> type);

   public static void checkStatusInNotNull(BehaviorTreeNodeStatus status)
   {
      if (status == null)
      {
         throw new RuntimeException("Behavior tree node status must not be null.");
      }
   }
}
