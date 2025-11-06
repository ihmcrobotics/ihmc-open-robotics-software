package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.BuildingExplorationState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeNode;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;

public class RDXBuildingExploration extends RDXBehaviorTreeNode<BuildingExplorationState, BuildingExplorationDefinition>
{
   public RDXBuildingExploration(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new BuildingExplorationState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      super.renderNodeSettingsWidgets();

      ImGui.text("Building Exploration Node");
   }
}
