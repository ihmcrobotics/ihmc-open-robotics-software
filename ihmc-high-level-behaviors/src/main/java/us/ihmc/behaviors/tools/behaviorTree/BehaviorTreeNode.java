package us.ihmc.behaviors.tools.behaviorTree;

import us.ihmc.commons.Conversions;

/**
 * The core interface of a Behavior Tree: the node that can be ticked.
 */
public abstract class BehaviorTreeNode implements BehaviorTreeNodeBasics
{
   private BehaviorTreeNodeStatus previousStatus = null;
   private long lastTickMillis = -1;
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
   public long getLastTickMillis()
   {
      return lastTickMillis;
   }

   public double getTimeSinceLastTick()
   {
      long lastTickMillis = getLastTickMillis();
      if (lastTickMillis == -1)
      {
         return Double.MAX_VALUE;
      }
      else
      {
         return Conversions.millisecondsToSeconds(System.currentTimeMillis() - lastTickMillis);
      }
   }

   public boolean wasTickedRecently(double maxTimeSince)
   {
      long lastTickMillis = getLastTickMillis();
      if (lastTickMillis == -1)
      {
         return false;
      }
      else
      {
         return Conversions.millisecondsToSeconds(System.currentTimeMillis() - lastTickMillis) < maxTimeSince;
      }
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
   public void setLastTickMillis(long lastTickMillis)
   {
      this.lastTickMillis = lastTickMillis;
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
