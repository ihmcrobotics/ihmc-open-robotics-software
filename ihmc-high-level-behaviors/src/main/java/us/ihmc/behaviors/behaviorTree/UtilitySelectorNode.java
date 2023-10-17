package us.ihmc.behaviors.behaviorTree;

public class UtilitySelectorNode extends LocalOnlyBehaviorTreeNodeExecutor
{
   public UtilitySelectorNode()
   {

   }

   @Override
   protected BehaviorTreeNodeStatus tickInternal()
   {
      double highestUtility = 0.0;
      BehaviorTreeNodeExecutor nodeOfHighestUtility = null;
      for (BehaviorTreeNodeExecutor child : getChildren())
      {
         double utility = child.getState().evaluateUtility();
         if (utility > highestUtility)
         {
            highestUtility = utility;
            nodeOfHighestUtility = child;
         }
      }

      if (highestUtility > 0.0)
      {
         return nodeOfHighestUtility.tick();
      }
      else
      {
         return BehaviorTreeNodeStatus.FAILURE;
      }
   }

   @Override
   public BehaviorTreeNodeDefinition getDefinition()
   {
      return null; // FIXME
   }
}
