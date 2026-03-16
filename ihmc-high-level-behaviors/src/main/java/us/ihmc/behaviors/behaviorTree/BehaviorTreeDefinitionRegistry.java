package us.ihmc.behaviors.behaviorTree;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.CheckpointNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.GotoNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceDefinition;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.*;

import java.util.HashMap;
import java.util.Map;

public class BehaviorTreeDefinitionRegistry
{
   private static final DefinitionMapping[] REGISTRY = new DefinitionMapping[]
   {
      new DefinitionMapping(BehaviorTreeRootNodeDefinition.class, BehaviorTreeStateMessage.ROOT_NODE),
      new DefinitionMapping(BehaviorTreeNodeDefinition.class, BehaviorTreeStateMessage.BASIC_NODE),
      new DefinitionMapping(ActionSequenceDefinition.class, BehaviorTreeStateMessage.ACTION_SEQUENCE),
      new DefinitionMapping(FallbackNodeDefinition.class, BehaviorTreeStateMessage.FALLBACK_NODE),
      new DefinitionMapping(ConditionNodeDefinition.class, BehaviorTreeStateMessage.CONDITION_NODE),
      new DefinitionMapping(GotoNodeDefinition.class, BehaviorTreeStateMessage.GOTO_NODE),
      new DefinitionMapping(CheckpointNodeDefinition.class, BehaviorTreeStateMessage.CHECKPOINT_NODE),
      new DefinitionMapping(SceneActionNodeDefinition.class, BehaviorTreeStateMessage.SCENE_ACTION),
      new DefinitionMapping(AI2RNodeDefinition.class, BehaviorTreeStateMessage.AI2R_NODE),
      new DefinitionMapping(DoorTraversalDefinition.class, BehaviorTreeStateMessage.DOOR_TRAVERSAL),
      new DefinitionMapping(BuildingExplorationDefinition.class, BehaviorTreeStateMessage.BUILDING_EXPLORATION),

      new DefinitionMapping(NeckActionDefinition.class, BehaviorTreeStateMessage.NECK_ACTION),
      new DefinitionMapping(SpineActionDefinition.class, BehaviorTreeStateMessage.SPINE_ACTION),
      new DefinitionMapping(WalkActionDefinition.class, BehaviorTreeStateMessage.WALK_ACTION),
      new DefinitionMapping(ArmActionDefinition.class, BehaviorTreeStateMessage.ARM_ACTION),
      new DefinitionMapping(HandWrenchActionDefinition.class, BehaviorTreeStateMessage.HAND_WRENCH_ACTION),
      new DefinitionMapping(ScrewPrimitiveActionDefinition.class, BehaviorTreeStateMessage.SCREW_PRIMITIVE_ACTION),
      new DefinitionMapping(PelvisActionDefinition.class, BehaviorTreeStateMessage.PELVIS_ACTION),
      new DefinitionMapping(AbilityHandActionDefinition.class, BehaviorTreeStateMessage.ABILITY_HAND_ACTION),
      new DefinitionMapping(EZGripperActionDefinition.class, BehaviorTreeStateMessage.EZGRIPPER_ACTION),
      new DefinitionMapping(WaitActionDefinition.class, BehaviorTreeStateMessage.WAIT_ACTION),
      new DefinitionMapping(LegActionDefinition.class, BehaviorTreeStateMessage.LEG_ACTION),
   };
   private static final Map<Class<?>, DefinitionMapping> REGISTRY_MAP = new HashMap<>();
   static
   {
      for (DefinitionMapping definitionEntry : REGISTRY)
      {
         REGISTRY_MAP.put(definitionEntry.getTypeClass(), definitionEntry);
      }
   }

   public static Class<?> getClassFromTypeName(String typeName)
   {
      for (DefinitionMapping definitionEntry : REGISTRY)
      {
         if (typeName.equals(definitionEntry.getTypeClass().getSimpleName()))
            return definitionEntry.getTypeClass();
      }
      return null;
   }

   public static Class<?> getNodeDefinitionClass(byte nodeType)
   {
      for (DefinitionMapping definitionEntry : REGISTRY)
      {
         if (nodeType == definitionEntry.getMessageByte())
            return definitionEntry.getTypeClass();
      }

      return null;
   }

   public static byte getMessageByte(Class<?> definitionClass)
   {
      DefinitionMapping definitionMapping = REGISTRY_MAP.get(definitionClass);
      if (definitionMapping != null)
      {
         return definitionMapping.getMessageByte();
      }

      return -1;
   }

   public static String getInitialName(Class<?> definitionClass)
   {
      DefinitionMapping definitionMapping = REGISTRY_MAP.get(definitionClass);
      if (definitionMapping != null)
      {
         return definitionMapping.getInitialName();
      }

      return null;
   }

   private static class DefinitionMapping
   {
      private final String initialName;
      private final Class<?> typeClass;
      private final byte messageByte;

      private DefinitionMapping(Class<?> typeClass, byte messageByte)
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
