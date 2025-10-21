package us.ihmc.behaviors.behaviorTree;

import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalDefinition;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.CheckPointNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.GotoNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceDefinition;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeDefinitionBuilder
{
   public static BehaviorTreeNodeDefinition createNode(Class<?> definitionType,
                                                       CRDTInfo crdtInfo,
                                                       WorkspaceResourceDirectory saveFileDirectory,
                                                       DefaultFootstepPlannerParametersReadOnly defaultFootstepPlannerParameters)
   {
      if (definitionType == BehaviorTreeNodeDefinition.class)
      {
         return new BehaviorTreeNodeDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == AI2RNodeDefinition.class)
      {
         return new AI2RNodeDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == ActionSequenceDefinition.class)
      {
         return new ActionSequenceDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == FallbackNodeDefinition.class)
      {
         return new FallbackNodeDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == ConditionNodeDefinition.class)
      {
         return new ConditionNodeDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == GotoNodeDefinition.class)
      {
         return new GotoNodeDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == CheckPointNodeDefinition.class)
      {
         return new CheckPointNodeDefinition(crdtInfo, saveFileDirectory);
      }
      if (definitionType == DoorTraversalDefinition.class)
      {
         return new DoorTraversalDefinition(crdtInfo, saveFileDirectory);
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
         return new FootstepPlanActionDefinition(crdtInfo, saveFileDirectory, new DefaultFootstepPlannerParameters(defaultFootstepPlannerParameters));
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
