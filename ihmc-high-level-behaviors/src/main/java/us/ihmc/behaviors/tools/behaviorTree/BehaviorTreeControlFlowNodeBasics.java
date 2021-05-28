package us.ihmc.behaviors.tools.behaviorTree;

/**
 * A behavior tree control flow node can have children, so provide the ability to add them.
 */
public interface BehaviorTreeControlFlowNodeBasics extends BehaviorTreeNodeBasics
{
   public abstract <T extends BehaviorTreeNodeBasics> T addChild(T child);
}
