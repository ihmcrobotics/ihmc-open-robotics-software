package us.ihmc.behaviors.buildingExploration;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BuildingExplorationExecutor extends BehaviorTreeNodeExecutor<BuildingExplorationState, BuildingExplorationDefinition>
{
   private final SceneGraph sceneGraph;

   public BuildingExplorationExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, SceneGraph sceneGraph)
   {
      super(new BuildingExplorationState(id, crdtInfo, saveFileDirectory));

      this.sceneGraph = sceneGraph;
   }

   // TODO: finish
   @Override
   public void update()
   {
      super.update();
   }
}
