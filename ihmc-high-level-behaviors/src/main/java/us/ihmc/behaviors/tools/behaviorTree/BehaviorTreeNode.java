package us.ihmc.behaviors.tools.behaviorTree;

import us.ihmc.robotics.time.TimeTools;

import java.time.Instant;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public abstract class BehaviorTreeNode implements BehaviorTreeNodeBasics
{
   private BehaviorTreeNodeStatus previousStatus = null;
   private Instant lastTickInstant = null;
   private String name = getClass().getSimpleName();
   private Class<?> type = BehaviorTreeNode.class;

   @Override
   public BehaviorTreeNodeStatus getPreviousStatus()
   {
      return previousStatus;
   }

   @Override
   public void setPreviousStatus(BehaviorTreeNodeStatus previousStatus)
   {
      this.previousStatus = previousStatus;
   }

   @Override
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

   public boolean wasTickedRecently(double maxTimeSince)
   {
      return hasBeenTicked() && TimeTools.calculateDelay(lastTickInstant) < maxTimeSince;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public void setName(String name)
   {
      this.name = name;
   }

   @Override
   public void setLastTickInstant(Instant lastTickInstant)
   {
      this.lastTickInstant = lastTickInstant;
   }

   @Override
   public void setType(Class<?> type)
   {
      this.type = type;
   }

   @Override
   public Class<?> getType()
   {
      return type;
   }
}
