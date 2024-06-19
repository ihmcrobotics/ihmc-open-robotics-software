package us.ihmc.behaviors.buildingExploration;

import behavior_msgs.msg.dds.BuildingExplorationStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.List;
import java.util.Stack;

public class BuildingExplorationState extends BehaviorTreeNodeState<BuildingExplorationDefinition>
{
   private final Stack<DoorNode> traversedDoorNodes = new Stack<>();
   private DoorNode nextDoorNode;

   public BuildingExplorationState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new BuildingExplorationDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   @Override
   public List<BehaviorTreeNodeState<?>> getChildren()
   {
      return super.getChildren();
   }

   public void toMessage(BuildingExplorationStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(BuildingExplorationStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }

   public Stack<DoorNode> getTraversedDoorNodes()
   {
      return traversedDoorNodes;
   }

   public DoorNode getNextDoorNode()
   {
      return nextDoorNode;
   }

   public void setNextDoorNode(DoorNode nextDoorNode)
   {
      this.nextDoorNode = nextDoorNode;
   }
}
