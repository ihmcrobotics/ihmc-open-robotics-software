package us.ihmc.behaviors.room;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RoomExplorationState extends BehaviorTreeNodeState<RoomExplorationDefinition>
{
   public RoomExplorationState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new RoomExplorationDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }
}
