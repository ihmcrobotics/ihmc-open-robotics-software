package us.ihmc.behaviors.room;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RoomExplorationExecutor extends BehaviorTreeNodeExecutor<RoomExplorationState, RoomExplorationDefinition>
{
   public RoomExplorationExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new RoomExplorationState(id, crdtInfo, saveFileDirectory));
   }
}
