package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;

import java.util.function.Consumer;

/**
 * This interface just exists to provide a better name to what this is,
 * which gets passed down from BehaviorTree's modifyTree method and serves
 * to queue up tree modifications.
 */
public interface BehaviorTreeModificationQueue<T extends BehaviorTreeNode> extends Consumer<BehaviorTreeModification<T>>
{

}
