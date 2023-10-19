package us.ihmc.behaviors.tools.behaviorTree;

import java.time.Instant;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public interface BehaviorTreeNodeBasics
{
   default double evaluateUtility()
   {
      return 1.0;
   }

   /**
    * A method that can be called on every node in the tree every time the root gets ticked
    * in order for parallel nodes to figure out when they are no longer being selected.
    */
   default void clock()
   {

   }

   default BehaviorTreeNodeStatus tick()
   {
      setPreviousStatus(tickInternal());
      setLastTickInstant(Instant.now());
      return getPreviousStatus();
   }

   BehaviorTreeNodeStatus tickInternal();

   /**
    * @return The node's status from the last time it was ticked.
    *         This will be null if the node hasn't been ticked yet.
    */
   BehaviorTreeNodeStatus getPreviousStatus();

   void setPreviousStatus(BehaviorTreeNodeStatus status);

   default boolean hasBeenTicked()
   {
      return getLastTickInstant() != null;
   }

   /**
    * @return The Instant at which this node was last ticked.
    *         This will be null if the node has never been ticked.
    */
   Instant getLastTickInstant();

   void setLastTickInstant(Instant lastTickInstant);

   String getName();

   void setName(String name);

   Class<?> getType();

   void setType(Class<?> type);

   static void checkStatusInNotNull(BehaviorTreeNodeStatus status)
   {
      if (status == null)
      {
         throw new RuntimeException("Behavior tree node status must not be null.");
      }
   }
}
