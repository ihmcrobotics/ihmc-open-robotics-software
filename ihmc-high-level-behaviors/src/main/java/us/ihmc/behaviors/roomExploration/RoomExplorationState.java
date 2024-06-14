package us.ihmc.behaviors.roomExploration;

import behavior_msgs.msg.dds.RoomExplorationStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.List;

public class RoomExplorationState extends BehaviorTreeNodeState<RoomExplorationDefinition>
{
   public RoomExplorationState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new RoomExplorationDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   @Override
   public List<BehaviorTreeNodeState<?>> getChildren()
   {
      return super.getChildren();
   }

   public void toMessage(RoomExplorationStateMessage stateMessage)
   {

   }
}
