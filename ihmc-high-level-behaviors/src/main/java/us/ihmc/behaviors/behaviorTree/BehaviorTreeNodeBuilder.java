package us.ihmc.behaviors.behaviorTree;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

/**
 * @param <T> The generic type of this node: RDX or Executor
 */
public interface BehaviorTreeNodeBuilder<T extends BehaviorTreeNode<T, ? ,?>>
{
   T createNode(Class<?> nodeType, long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory);
}
