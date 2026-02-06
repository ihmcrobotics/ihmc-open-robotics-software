package us.ihmc.behaviors.behaviorTree.control.buildingExploration;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;

public class BuildingExplorationExecutor extends BehaviorTreeNodeExecutor<BuildingExplorationState, BuildingExplorationDefinition>
{
   public BuildingExplorationExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new BuildingExplorationState(id, rootNode.getState()), rootNode);
   }

   // TODO: finish
   @Override
   public void update()
   {
      super.update();
   }
}
