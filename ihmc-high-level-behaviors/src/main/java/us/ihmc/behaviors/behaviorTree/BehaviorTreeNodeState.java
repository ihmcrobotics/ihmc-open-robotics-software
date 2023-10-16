package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.robotics.time.TimeTools;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public abstract class BehaviorTreeNodeState implements BehaviorTreeNodeDefinitionSupplier
{
   /** The current status of the behavior tree node. */
   private BehaviorTreeNodeStatus status = BehaviorTreeNodeStatus.NOT_TICKED;
   /** TODO: What is this used for and do we need this? */
   private Instant lastTickInstant = null;

   private final List<BehaviorTreeNodeState> children = new ArrayList<>();

   /**
    * A method that can be called on every node in the tree every time the root gets ticked
    * in order for parallel nodes to figure out when they are no longer being selected.
    *
    * TODO: Perhaps this should just be update?
    */
   public void clock()
   {
      for (BehaviorTreeNodeState child : children)
      {
         child.clock();
      }
   }

   public void toMessage(BehaviorTreeNodeStateMessage message)
   {
      message.setStatus(status.toByte());
      if (lastTickInstant != null)
         MessageTools.toMessage(lastTickInstant, message.getLastTickInstant());
   }

   public void fromMessage(BehaviorTreeNodeStateMessage message)
   {
      status = BehaviorTreeNodeStatus.fromByte(message.getStatus());
      if (message.getLastTickInstant().getSecondsSinceEpoch() != 0L || message.getLastTickInstant().getAdditionalNanos() != 0L)
         lastTickInstant = MessageTools.toInstant(message.getLastTickInstant());
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

   public List<BehaviorTreeNodeState> getChildren()
   {
      return children;
   }
}
