package us.ihmc.behaviors.behaviorTree;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

/**
 * States won't be built directly. They'll be Executors or UI types.
 */
public interface BehaviorTreeNodeStateBuilder
{
   BehaviorTreeNodeLayer createNode(Class<?> nodeType, long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory);
}
