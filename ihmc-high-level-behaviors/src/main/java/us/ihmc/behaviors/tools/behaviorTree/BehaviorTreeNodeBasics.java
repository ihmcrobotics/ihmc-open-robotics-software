package us.ihmc.behaviors.tools.behaviorTree;

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
      setLastTickMillis(System.currentTimeMillis());
      return getPreviousStatus();
   }

   public abstract BehaviorTreeNodeStatus tickInternal();

   public abstract BehaviorTreeNodeStatus getPreviousStatus();

   public abstract void setPreviousStatus(BehaviorTreeNodeStatus status);

   public abstract long getLastTickMillis();

   public abstract void setLastTickMillis(long lastTickMillis);

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
