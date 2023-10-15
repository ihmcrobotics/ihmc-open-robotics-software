package us.ihmc.behaviors.behaviorTree;

import us.ihmc.robotics.time.TimeTools;

import java.time.Instant;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public abstract class BehaviorTreeNodeState
{
   private BehaviorTreeNodeStatus previousStatus = null;
   private Instant lastTickInstant = null;
   private String name = getClass().getSimpleName();


   public BehaviorTreeNodeStatus tick()
   {
      setPreviousStatus(tickInternal());
      setLastTickInstant(Instant.now());
      return getStatus();
   }


   public abstract BehaviorTreeNodeStatus tickInternal();

   /**
    * A method that can be called on every node in the tree every time the root gets ticked
    * in order for parallel nodes to figure out when they are no longer being selected.
    */
   public void clock()
   {

   }

   /**
    * @return The node's status from the last time it was ticked.
    *         This will be null if the node hasn't been ticked yet.
    */
   public BehaviorTreeNodeStatus getStatus()
   {
      return previousStatus;
   }

   public void setPreviousStatus(BehaviorTreeNodeStatus previousStatus)
   {
      this.previousStatus = previousStatus;
   }

   /**
    * @return The Instant at which this node was last ticked.
    *         This will be null if the node has never been ticked.
    */
   public Instant getLastTickInstant()
   {
      return lastTickInstant;
   }

   public double getTimeSinceLastTick()
   {
      if (hasBeenTicked())
         return TimeTools.calculateDelay(lastTickInstant);
      else
         return Double.NaN;
   }

   public boolean hasBeenTicked()
   {
      return getLastTickInstant() != null;
   }

   public boolean wasTickedRecently(double maxTimeSince)
   {
      return hasBeenTicked() && TimeTools.calculateDelay(lastTickInstant) < maxTimeSince;
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setLastTickInstant(Instant lastTickInstant)
   {
      this.lastTickInstant = lastTickInstant;
   }

   public double evaluateUtility()
   {
      return 1.0;
   }

   static void checkStatusIsNotNull(BehaviorTreeNodeStatus status)
   {
      if (status == null)
      {
         throw new RuntimeException("Behavior tree node status must not be null.");
      }
   }
}
