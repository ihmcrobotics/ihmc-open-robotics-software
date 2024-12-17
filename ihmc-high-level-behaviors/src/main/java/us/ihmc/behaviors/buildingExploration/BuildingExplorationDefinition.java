package us.ihmc.behaviors.buildingExploration;

import behavior_msgs.msg.dds.BuildingExplorationDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.communication.crdt.CRDTGlobalInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BuildingExplorationDefinition extends BehaviorTreeNodeDefinition
{
   public BuildingExplorationDefinition(CRDTGlobalInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);
   }

   public void toMessage(BuildingExplorationDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());
   }

   public void fromMessage(BuildingExplorationDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());
   }
}
