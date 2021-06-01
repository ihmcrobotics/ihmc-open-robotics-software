package us.ihmc.behaviors.tools.behaviorTree;

/**
 * A fallback node proceeds through children left to right until they return SUCCESS.
 */
public class FallbackNode extends BehaviorTreeControlFlowNode implements FallbackNodeBasics
{
   public FallbackNode()
   {
      setType(FallbackNode.class);
   }
}
