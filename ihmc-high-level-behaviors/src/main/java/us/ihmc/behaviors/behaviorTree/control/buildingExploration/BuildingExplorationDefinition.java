package us.ihmc.behaviors.behaviorTree.control.buildingExploration;

import behavior_msgs.msg.dds.BuildingExplorationDefinitionMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;

public class BuildingExplorationDefinition extends BehaviorTreeNodeDefinition
{
   public BuildingExplorationDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);
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
