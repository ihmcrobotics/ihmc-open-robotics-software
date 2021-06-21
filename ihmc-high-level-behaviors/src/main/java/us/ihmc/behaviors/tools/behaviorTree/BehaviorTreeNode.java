package us.ihmc.behaviors.tools.behaviorTree;

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
