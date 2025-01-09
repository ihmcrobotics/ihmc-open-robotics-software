package us.ihmc.behaviors.behaviorTree;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

/**
 * @param <HLT> The generic type of this node: RDX or Executor
 */
public interface BehaviorTreeNodeBuilder<HLT extends BehaviorTreeNode<HLT, ? ,?>>
{
   HLT createNode(Class<?> nodeType, long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory);
}
