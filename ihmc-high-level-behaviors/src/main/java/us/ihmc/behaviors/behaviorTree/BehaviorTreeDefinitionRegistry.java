package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionDefinition;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.*;

public class BehaviorTreeDefinitionRegistry
{
   private static final RegistryRecord[] DEFINITIONS = new RegistryRecord[]
   {
      new RegistryRecord(BehaviorTreeRootNodeDefinition.class, BehaviorTreeStateMessage.ROOT_NODE),
      new RegistryRecord(BehaviorTreeNodeDefinition.class, BehaviorTreeStateMessage.BASIC_NODE),
      new RegistryRecord(AI2RNodeDefinition.class, BehaviorTreeStateMessage.AI2R_NODE),
      new RegistryRecord(ActionSequenceDefinition.class, BehaviorTreeStateMessage.ACTION_SEQUENCE),
      new RegistryRecord(DoorTraversalDefinition.class, BehaviorTreeStateMessage.DOOR_TRAVERSAL),
      new RegistryRecord(TrashCanInteractionDefinition.class, BehaviorTreeStateMessage.TRASH_CAN_INTERACTION),
      new RegistryRecord(BuildingExplorationDefinition.class, BehaviorTreeStateMessage.BUILDING_EXPLORATION),

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
         if (typeName.equals(definitionEntry.getTypeClass().getSimpleName()))
            return definitionEntry.getTypeClass();
      }

      return null;
   }

   public static Class<?> getNodeDefinitionClass(byte nodeType)
   {
      for (RegistryRecord definitionEntry : DEFINITIONS)
      {
         if (nodeType == definitionEntry.getMessageByte())
            return definitionEntry.getTypeClass();
      }

      return null;
   }

   public static String getInitialName(Class<?> definitionClass)
   {
      for (RegistryRecord definitionEntry : DEFINITIONS)
      {
         if (definitionClass == definitionEntry.getTypeClass())
            return definitionEntry.getInitialName();
      }

      return null;
   }

   private static class RegistryRecord
   {
      private final String initialName;
      private final Class<?> typeClass;
      private final byte messageByte;

      private RegistryRecord(Class<?> typeClass, byte messageByte)
      {
         this.typeClass = typeClass;
         this.messageByte = messageByte;

         if (typeClass == BehaviorTreeNodeDefinition.class)
         {
            this.initialName = "Basic node";
         }
         else
         {
            // Convert "PascalCase" to "Sentence case"
            String initialName = typeClass.getSimpleName();
            initialName = initialName.replaceFirst("^BehaviorTree", "");
            initialName = initialName.replaceFirst("Definition$", "");
            initialName = initialName.replaceFirst("Node$", " Node");
            initialName = initialName.replaceAll("([a-z])([A-Z]+)", "$1 $2");

            // Undercase words after first
            String[] words = initialName.split(" ");
            if (words.length > 1)
            {
               StringBuilder modifiedName = new StringBuilder();
               modifiedName.append(words[0]);
               for (int i = 1; i < words.length; i++)
               {
                  modifiedName.append(" ").append(words[i].toLowerCase());
               }
               initialName = modifiedName.toString();
            }

            this.initialName = initialName;
         }
      }

      public String getInitialName()
      {
         return initialName;
      }

      public Class<?> getTypeClass()
      {
         return typeClass;
      }

      public byte getMessageByte()
      {
         return messageByte;
      }
   }
}
