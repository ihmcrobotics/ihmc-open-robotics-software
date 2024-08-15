package us.ihmc.behaviors.behaviorTree;

import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionDefinition;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeDefinitionBuilder
{
   public static BehaviorTreeNodeDefinition createNode(Class<?> definitionType, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      if (definitionType == BehaviorTreeNodeDefinition.class)
      {
         return new BehaviorTreeNodeDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == ActionSequenceDefinition.class)
      {
         return new ActionSequenceDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == DoorTraversalDefinition.class)
      {
         return new DoorTraversalDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == TrashCanInteractionDefinition.class)
      {
         return new TrashCanInteractionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == BuildingExplorationDefinition.class)
      {
         return new BuildingExplorationDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == ChestOrientationActionDefinition.class)
      {
         return new ChestOrientationActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == FootstepPlanActionDefinition.class)
      {
         return new FootstepPlanActionDefinition(crdtInfo, saveFileDirectory, new DefaultFootstepPlannerParameters());
      }
      if (definitionType == KickDoorApproachPlanActionDefinition.class)
      {
         return new KickDoorApproachPlanActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == HandPoseActionDefinition.class)
      {
         return new HandPoseActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == HandWrenchActionDefinition.class)
      {
         return new HandWrenchActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == PelvisHeightOrientationActionDefinition.class)
      {
         return new PelvisHeightOrientationActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == SakeHandCommandActionDefinition.class)
      {
         return new SakeHandCommandActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == ScrewPrimitiveActionDefinition.class)
      {
         return new ScrewPrimitiveActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == WaitDurationActionDefinition.class)
      {
         return new WaitDurationActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == FootPoseActionDefinition.class)
      {
         return new FootPoseActionDefinition(crdtInfo, saveFileDirectory);
      }

      throw new RuntimeException("Node definition type not found: " + definitionType.getSimpleName());
   }
}
