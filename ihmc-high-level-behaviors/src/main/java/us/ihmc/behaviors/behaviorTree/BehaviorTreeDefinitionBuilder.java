package us.ihmc.behaviors.behaviorTree;

import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.roomExploration.RoomExplorationDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionDefinition;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionDefinition;
import us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition;
import us.ihmc.behaviors.sequence.actions.HandWrenchActionDefinition;
import us.ihmc.behaviors.sequence.actions.PelvisHeightPitchActionDefinition;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionDefinition;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionDefinition;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
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
      if (definitionType == RoomExplorationDefinition.class)
      {
         return new RoomExplorationDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == ChestOrientationActionDefinition.class)
      {
         return new ChestOrientationActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == FootstepPlanActionDefinition.class)
      {
         return new FootstepPlanActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == HandPoseActionDefinition.class)
      {
         return new HandPoseActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == HandWrenchActionDefinition.class)
      {
         return new HandWrenchActionDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == PelvisHeightPitchActionDefinition.class)
      {
         return new PelvisHeightPitchActionDefinition(crdtInfo, saveFileDirectory);
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

      throw new RuntimeException("Node definition type not found: " + definitionType.getSimpleName());
   }
}
