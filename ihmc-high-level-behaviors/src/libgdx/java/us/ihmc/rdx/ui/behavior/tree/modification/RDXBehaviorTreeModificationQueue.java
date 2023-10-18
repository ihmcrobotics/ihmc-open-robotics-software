package us.ihmc.rdx.ui.behavior.tree.modification;

import java.util.function.Consumer;

/**
 * This interface just exists to provide a better name to what this is,
 * which gets passed down from BehaviorTree's modifyTree method and serves
 * to queue up tree modifications.
 */
public interface RDXBehaviorTreeModificationQueue extends Consumer<RDXBehaviorTreeModification>
{

}
