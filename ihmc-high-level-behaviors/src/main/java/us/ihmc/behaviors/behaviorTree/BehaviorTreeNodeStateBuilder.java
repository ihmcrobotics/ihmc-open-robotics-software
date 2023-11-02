package us.ihmc.behaviors.behaviorTree;

import us.ihmc.communication.crdt.CRDTInfo;

/**
 * States won't be built directly. They'll be Executors or UI types.
 */
public interface BehaviorTreeNodeStateBuilder
{
   BehaviorTreeNodeExtension createNode(Class<?> nodeType, long id, CRDTInfo crdtInfo);
}
