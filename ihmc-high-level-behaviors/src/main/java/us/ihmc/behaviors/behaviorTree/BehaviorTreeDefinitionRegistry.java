package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionDefinition;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.*;

public class BehaviorTreeDefinitionRegistry
{
   private record RegistryRecord(Class<?> typeClass, byte messageByte) { }

   private static final RegistryRecord[] DEFINITIONS = new RegistryRecord[]
   {
      new RegistryRecord(BehaviorTreeNodeDefinition.class, BehaviorTreeStateMessage.BASIC_NODE),
      new RegistryRecord(ActionSequenceDefinition.class, BehaviorTreeStateMessage.ACTION_SEQUENCE),
      new RegistryRecord(DoorTraversalDefinition.class, BehaviorTreeStateMessage.DOOR_TRAVERSAL),
      new RegistryRecord(TrashCanInteractionDefinition.class, BehaviorTreeStateMessage.TRASH_CAN_INTERACTION),

      new RegistryRecord(ChestOrientationActionDefinition.class, BehaviorTreeStateMessage.CHEST_ORIENTATION_ACTION),
      new RegistryRecord(FootstepPlanActionDefinition.class, BehaviorTreeStateMessage.FOOTSTEP_PLAN_ACTION),
      new RegistryRecord(HandPoseActionDefinition.class, BehaviorTreeStateMessage.HAND_POSE_ACTION),
      new RegistryRecord(HandWrenchActionDefinition.class, BehaviorTreeStateMessage.HAND_WRENCH_ACTION),
      new RegistryRecord(ScrewPrimitiveActionDefinition.class, BehaviorTreeStateMessage.SCREW_PRIMITIVE_ACTION),
      new RegistryRecord(PelvisHeightOrientationActionDefinition.class, BehaviorTreeStateMessage.PELVIS_HEIGHT_ORIENTATION_ACTION),
      new RegistryRecord(SakeHandCommandActionDefinition.class, BehaviorTreeStateMessage.SAKE_HAND_COMMAND_ACTION),
      new RegistryRecord(WaitDurationActionDefinition.class, BehaviorTreeStateMessage.WAIT_DURATION_ACTION),
      new RegistryRecord(FootPoseActionDefinition.class, BehaviorTreeStateMessage.FOOT_POSE_ACTION)
   };

   public static Class<?> getClassFromTypeName(String typeName)
   {
      for (RegistryRecord definitionEntry : DEFINITIONS)
      {
         if (typeName.equals(definitionEntry.typeClass().getSimpleName()))
            return definitionEntry.typeClass();
      }

      return null;
   }

   public static Class<?> getNodeStateClass(byte nodeType)
   {
      for (RegistryRecord definitionEntry : DEFINITIONS)
      {
         if (nodeType == definitionEntry.messageByte())
            return definitionEntry.typeClass();
      }

      return null;
   }
}
