package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import com.esotericsoftware.kryo.util.IntMap;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.*;

import java.util.HashMap;

public enum BehaviorTreeDefinitionRegistry
{
   BASIC_NODE(BehaviorTreeNodeDefinition.class),
   ACTION_SEQUENCE(ActionSequenceDefinition.class),
   DOOR_TRAVERSAL(DoorTraversalDefinition.class),
   CHEST_ORIENTATION_ACTION(ChestOrientationActionDefinition.class),
   FOOTSTEP_PLAN_ACTION(FootstepPlanActionDefinition.class),
   HAND_POSE_ACTION(HandPoseActionDefinition.class),
   HAND_WRENCH_ACTION(HandWrenchActionDefinition.class),
   SCREW_PRIMITIVE_ACTION(ScrewPrimitiveActionDefinition.class),
   PELVIS_HEIGHT_PITCH_ACTION(PelvisHeightPitchActionDefinition.class),
   SAKE_HAND_COMMAND_ACTION(SakeHandCommandActionDefinition.class),
   WAIT_DURATION_ACTION(WaitDurationActionDefinition.class),
   KICK_DOOR_ACTION(KickDoorActionDefinition.class),
   KICK_DOOR_APPROACH_ACTION(KickDoorApproachPlanActionDefinition.class);

   private final Class<?> typeClass;

   private BehaviorTreeDefinitionRegistry(Class<?> typeClass)
   {
      this.typeClass = typeClass;
   }

   public static BehaviorTreeDefinitionRegistry[] values = values();

   public static BehaviorTreeDefinitionRegistry fromByte(byte index)
   {
      return values[index];
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public Class<?> getTypeClass()
   {
      return typeClass;
   }

   public static BehaviorTreeDefinitionRegistry getTypeFromClass(Class<?> clazz)
   {
      for (BehaviorTreeDefinitionRegistry type : values)
      {
         if (type.getTypeClass().equals(clazz))
            return type;
      }

      return null;
   }

   public static Class<?> getClassFromTypeName(String typeName)
   {
      for (BehaviorTreeDefinitionRegistry definitionType : values)
      {
         if (typeName.equals(definitionType.getTypeClass().getSimpleName()))
            return definitionType.typeClass;
      }

      return null;
   }

   public static Class<?> getNodeStateClass(byte nodeType)
   {
      return fromByte(nodeType).getTypeClass();
   }
}
