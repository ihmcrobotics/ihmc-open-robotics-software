package us.ihmc.behaviors.behaviorTree;

/**
 * A fallback node proceeds through children left to right until they return SUCCESS.
 */
public class FallbackNode extends BehaviorTreeControlFlowNode implements FallbackNodeBasics
{
   public FallbackNode()
   {

   }
}
