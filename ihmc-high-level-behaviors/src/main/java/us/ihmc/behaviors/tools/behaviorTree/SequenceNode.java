package us.ihmc.behaviors.tools.behaviorTree;

/**
 * A sequence node proceeds through children left to right while they are SUCCESSful.
 */
public class SequenceNode extends BehaviorTreeControlFlowNode implements SequenceNodeBasics
{
   public SequenceNode()
   {
      setType(SequenceNode.class);
   }
}
