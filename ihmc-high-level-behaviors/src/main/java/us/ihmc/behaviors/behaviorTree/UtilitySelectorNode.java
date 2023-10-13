package us.ihmc.behaviors.behaviorTree;

public class UtilitySelectorNode extends BehaviorTreeControlFlowNode
{
   public UtilitySelectorNode()
   {

   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      double highestUtility = 0.0;
      BehaviorTreeNodeBasics nodeOfHighestUtility = null;
      for (BehaviorTreeNodeBasics child : getChildren())
      {
         double utility = child.evaluateUtility();
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
}
