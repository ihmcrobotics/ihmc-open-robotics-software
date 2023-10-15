package us.ihmc.behaviors.behaviorTree;

/**
 * This node exists for the UI side tree that is updated
 * via network communication. It is used to show a visualization
 * of the tree's state and NOT to perform any computation.
 * All nodes in the UI side tree are instances of this class.
 */
public class BehaviorTreeStatusNode extends BehaviorTreeControlFlowNode
{
   /**
    * Normal use.
    */
   public BehaviorTreeStatusNode()
   {
   }

   /**
    * Deep copy constuctor.
    */
   public BehaviorTreeStatusNode(BehaviorTreeNodeState node)
   {
      setPreviousStatus(node.getStatus());
      setName(node.getName());
      setLastTickInstant(node.getLastTickInstant());

      if (node instanceof BehaviorTreeControlFlowNode controlFlowNode)
      {
         setHasBeenClocked(controlFlowNode.getHasBeenClocked());

         for (BehaviorTreeNodeState child : controlFlowNode.getChildren())
         {
            addChild(new BehaviorTreeStatusNode(child));
         }
      }
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      return BehaviorTreeNodeStatus.SUCCESS;
   }
}