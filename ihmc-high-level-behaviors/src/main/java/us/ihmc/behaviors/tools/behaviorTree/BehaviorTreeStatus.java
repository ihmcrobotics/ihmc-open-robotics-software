package us.ihmc.behaviors.tools.behaviorTree;

/**
 * This node exists for the UI side tree that is updated
 * via network communication. It is used to show a visualization
 * of the tree's state and NOT to perform any computation.
 * All nodes in the UI side tree are instances of this class.
 */
public class BehaviorTreeStatus extends BehaviorTreeControlFlowNode
{
   /**
    * Normal use.
    */
   public BehaviorTreeStatus()
   {
   }

   /**
    * Deep copy constuctor.
    */
   public BehaviorTreeStatus(BehaviorTreeNodeBasics node)
   {
      setPreviousStatus(node.getPreviousStatus());
      setName(node.getName());
      setLastTickInstant(node.getLastTickInstant());
      setType(node.getType());

      if (node instanceof BehaviorTreeControlFlowNodeBasics controlFlowNode)
      {
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