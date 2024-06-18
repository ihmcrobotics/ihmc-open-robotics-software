package us.ihmc.behaviors.buildingExploration;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.door.DoorTraversalExecutor;
import us.ihmc.behaviors.sequence.ActionSequenceExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
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

      // Find the next door to traverse
      if (getState().getNextDoorNode() == null)
      {
         for (SceneNode sceneNode : sceneGraph.getSceneNodesByID())
         {
            if (sceneNode instanceof DoorNode doorNode)
            {
               // TODO: check detection status
               // boolean detected = doorNode.isDetected();
               boolean detected = true;

               // If detected and we have not traversed the door
               if (detected && !getState().getTraversedDoorNodes().contains(doorNode))
               {
                  getState().setNextDoorNode(doorNode);
               }
            }
         }
      }

      for (BehaviorTreeNodeExecutor<?, ?> child : getChildren())
      {
         if (child instanceof DoorTraversalExecutor doorTraversalExecutor)
         {
            doorTraversalExecutor.getState().setDoorNode(getState().getNextDoorNode());
         }
      }
   }
}
