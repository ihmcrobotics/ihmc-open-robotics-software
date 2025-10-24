package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.behaviors.behaviorTree.condition.*;
import us.ihmc.behaviors.behaviorTree.control.*;
import us.ihmc.behaviors.behaviorTree.control.ai2r.*;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.*;
import us.ihmc.behaviors.behaviorTree.control.door.*;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class BehaviorTreeDefinitionBuilder
{
   public static BehaviorTreeRootNodeDefinition createRootNode(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, DRCRobotModel robotModel)
   {
      return new BehaviorTreeRootNodeDefinition(crdtInfo, saveFileDirectory, robotModel);
   }

   public static BehaviorTreeNodeDefinition createNode(Class<?> definitionType, BehaviorTreeRootNodeDefinition rootNode)
   {
      if (definitionType == BehaviorTreeNodeDefinition.class)
         return new BehaviorTreeNodeDefinition(rootNode);
      if (definitionType == AI2RNodeDefinition.class)
         return new AI2RNodeDefinition(rootNode);
      if (definitionType == ActionSequenceDefinition.class)
         return new ActionSequenceDefinition(rootNode);
      if (definitionType == FallbackNodeDefinition.class)
         return new FallbackNodeDefinition(rootNode);
      if (definitionType == ConditionNodeDefinition.class)
         return new ConditionNodeDefinition(rootNode);
      if (definitionType == GotoNodeDefinition.class)
         return new GotoNodeDefinition(rootNode);
      if (definitionType == CheckPointNodeDefinition.class)
         return new CheckPointNodeDefinition(rootNode);
      if (definitionType == DoorTraversalDefinition.class)
         return new DoorTraversalDefinition(rootNode);
      if (definitionType == BuildingExplorationDefinition.class)
         return new BuildingExplorationDefinition(rootNode);
      if (definitionType == ChestOrientationActionDefinition.class)
         return new ChestOrientationActionDefinition(rootNode);
      if (definitionType == FootstepPlanActionDefinition.class)
         return new FootstepPlanActionDefinition(rootNode);
      if (definitionType == HandPoseActionDefinition.class)
         return new HandPoseActionDefinition(rootNode);
      if (definitionType == HandWrenchActionDefinition.class)
         return new HandWrenchActionDefinition(rootNode);
      if (definitionType == PelvisHeightOrientationActionDefinition.class)
         return new PelvisHeightOrientationActionDefinition(rootNode);
      if (definitionType == SakeHandCommandActionDefinition.class)
         return new SakeHandCommandActionDefinition(rootNode);
      if (definitionType == ScrewPrimitiveActionDefinition.class)
         return new ScrewPrimitiveActionDefinition(rootNode);
      if (definitionType == WaitDurationActionDefinition.class)
         return new WaitDurationActionDefinition(rootNode);
      if (definitionType == FootPoseActionDefinition.class)
         return new FootPoseActionDefinition(rootNode);

      throw new RuntimeException("Node definition type not found: " + definitionType.getSimpleName());
   }
}
