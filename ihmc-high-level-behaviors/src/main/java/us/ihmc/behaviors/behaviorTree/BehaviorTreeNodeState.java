package us.ihmc.behaviors.behaviorTree;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.time.TimeTools;

import java.time.Instant;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public class BehaviorTreeNodeState implements BehaviorTreeNodeDefinitionSupplier
{
   private final BehaviorTreeNodeDefinition definition = new BehaviorTreeNodeDefinition();

   /** The current status of the behavior tree node. */
   private BehaviorTreeNodeStatus status = BehaviorTreeNodeStatus.NOT_TICKED;
   private Instant lastTickInstant = null;

   private final RecyclingArrayList<BehaviorTreeNodeState> children = new RecyclingArrayList<>(BehaviorTreeNodeState::new);

   public BehaviorTreeNodeState()
   {
   }

   public BehaviorTreeNodeStatus tick()
   {
      status = tickInternal();
      setLastTickInstant(Instant.now());
      return getStatus();
   }

   // TODO: Is this a good convention doing the *Internal thing?
   //   or would it be better to call super.tick or something somehow
   public BehaviorTreeNodeStatus tickInternal()
   {

   }

   /**
    * A method that can be called on every node in the tree every time the root gets ticked
    * in order for parallel nodes to figure out when they are no longer being selected.
    *
    * TODO: Perhaps this should just be update?
    */
   public void clock()
   {

   }

   public void setStatus(BehaviorTreeNodeStatus status)
   {
      this.status = status;
   }

   /**
    * @return The node's status from the last time it was ticked.
    *         This will be null if the node hasn't been ticked yet.
    */
   public BehaviorTreeNodeStatus getStatus()
   {
      return status;
   }

   public void setLastTickInstant(Instant lastTickInstant)
   {
      this.lastTickInstant = lastTickInstant;
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
      return lastTickInstant != null;
   }

   public boolean wasTickedRecently(double maxTimeSince)
   {
      return hasBeenTicked() && TimeTools.calculateDelay(lastTickInstant) < maxTimeSince;
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

   @Override
   public BehaviorTreeNodeDefinition getDefinition()
   {
      return definition;
   }
}
