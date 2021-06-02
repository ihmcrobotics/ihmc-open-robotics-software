package us.ihmc.behaviors.tools.behaviorTree;

public class BehaviorTreeStatus extends BehaviorTreeControlFlowNode
{
   public BehaviorTreeStatus()
   {
   }

   public BehaviorTreeStatus(BehaviorTreeNodeBasics node)
   {
      setPreviousStatus(node.getPreviousStatus());
      setName(node.getName());
      setLastTickMillis(node.getLastTickMillis());
      setType(node.getType());

      if (node instanceof BehaviorTreeControlFlowNodeBasics)
      {
         BehaviorTreeControlFlowNodeBasics controlFlowNode = (BehaviorTreeControlFlowNodeBasics) node;

         setHasBeenClocked(controlFlowNode.getHasBeenClocked());

         for (BehaviorTreeNodeBasics child : controlFlowNode.getChildren())
         {
            addChild(new BehaviorTreeStatus(child));
         }
      }
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      return BehaviorTreeNodeStatus.SUCCESS;
   }
}
