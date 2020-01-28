package us.ihmc.humanoidBehaviors.tools.behaviorTree;

public interface BehaviorTreeControlFlowNode extends BehaviorTreeNode
{
   <T extends BehaviorTreeNode> T addChild(T child);
}
