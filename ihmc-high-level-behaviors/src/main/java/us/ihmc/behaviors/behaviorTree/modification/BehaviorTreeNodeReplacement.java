package us.ihmc.behaviors.behaviorTree.modification;

/**
 * An actionable behavior tree node replacement back into the tree,
 * used primary for network synchronized subscriptions where the tree
 * is disassembled entirely and put back together on every newly
 * recieved message.
 *
 * In this case, you don't want to freeze the parent and you don't
 * need to recursively check children frames, as the tree is rebuilt
 * in depth first order.
 */
public class BehaviorTreeNodeReplacement
{
}
