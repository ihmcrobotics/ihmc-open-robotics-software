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

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class BehaviorTreeDefinitionBuilder
{
   private static final Map<Class<?>, Function<BehaviorTreeRootNodeDefinition, BehaviorTreeNodeDefinition>> MAP = new HashMap<>();
   static
   {
      MAP.put(BehaviorTreeNodeDefinition.class, BehaviorTreeNodeDefinition::new);
      MAP.put(ActionSequenceDefinition.class, ActionSequenceDefinition::new);
      MAP.put(FallbackNodeDefinition.class, FallbackNodeDefinition::new);
      MAP.put(ConditionNodeDefinition.class, ConditionNodeDefinition::new);
      MAP.put(GotoNodeDefinition.class, GotoNodeDefinition::new);
      MAP.put(CheckPointNodeDefinition.class, CheckPointNodeDefinition::new);
      MAP.put(SceneActionNodeDefinition.class, SceneActionNodeDefinition::new);
      MAP.put(AI2RNodeDefinition.class, AI2RNodeDefinition::new);
      MAP.put(DoorTraversalDefinition.class, DoorTraversalDefinition::new);
      MAP.put(BuildingExplorationDefinition.class, BuildingExplorationDefinition::new);
      MAP.put(NeckActionDefinition.class, NeckActionDefinition::new);
      MAP.put(SpineActionDefinition.class, SpineActionDefinition::new);
      MAP.put(WalkActionDefinition.class, WalkActionDefinition::new);
      MAP.put(ArmActionDefinition.class, ArmActionDefinition::new);
      MAP.put(HandWrenchActionDefinition.class, HandWrenchActionDefinition::new);
      MAP.put(PelvisActionDefinition.class, PelvisActionDefinition::new);
      MAP.put(AbilityHandActionDefinition.class, AbilityHandActionDefinition::new);
      MAP.put(SakeHandCommandActionDefinition.class, SakeHandCommandActionDefinition::new);
      MAP.put(ScrewPrimitiveActionDefinition.class, ScrewPrimitiveActionDefinition::new);
      MAP.put(WaitDurationActionDefinition.class, WaitDurationActionDefinition::new);
      MAP.put(LegActionDefinition.class, LegActionDefinition::new);
   }

   public static BehaviorTreeRootNodeDefinition createRootNode(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, DRCRobotModel robotModel)
   {
      return new BehaviorTreeRootNodeDefinition(crdtInfo, saveFileDirectory, robotModel);
   }

   public static BehaviorTreeNodeDefinition createNode(Class<?> definitionType, BehaviorTreeRootNodeDefinition rootNode)
   {
      if (MAP.containsKey(definitionType))
         return MAP.get(definitionType).apply(rootNode);

      throw new RuntimeException("Node definition type not found: " + definitionType.getSimpleName());
   }
}
