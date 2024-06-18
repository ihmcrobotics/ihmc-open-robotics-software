package us.ihmc.behaviors.buildingExploration;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BuildingExplorationExecutor extends BehaviorTreeNodeExecutor<BuildingExplorationState, BuildingExplorationDefinition>
{
   public BuildingExplorationExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new BuildingExplorationState(id, crdtInfo, saveFileDirectory));
   }

   @Override
   public void clock()
   {
      super.clock();
   }

   @Override
   public void tick()
   {
      super.tick();
   }

   @Override
   public void update()
   {
      super.update();
   }
}
