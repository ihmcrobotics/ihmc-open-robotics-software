package us.ihmc.rdx.behaviorTree;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.rdx.behaviorTree.actions.RDXAbilityHandAction;
import us.ihmc.rdx.behaviorTree.actions.RDXArmAction;
import us.ihmc.rdx.behaviorTree.actions.RDXEZGripperAction;
import us.ihmc.rdx.behaviorTree.actions.RDXNeckAction;
import us.ihmc.rdx.behaviorTree.actions.RDXSpineAction;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry.getClassFromTypeName;

public class RDXBehaviorNodeDuplication
{
   private final RDXBehaviorTree behaviorTree;

   public RDXBehaviorNodeDuplication(RDXBehaviorTree behaviorTree)
   {
      this.behaviorTree = behaviorTree;
   }

   public boolean supportsInvariantMirroring(RDXBehaviorTreeNode<?, ?> node)
   {
      boolean supports = false;
      supports |= node instanceof RDXArmAction armAction && armAction.getDefinition().getUsePredefinedJointAngles();
      supports |= node instanceof RDXAbilityHandAction;
      supports |= node instanceof RDXEZGripperAction;
      supports |= node instanceof RDXNeckAction;
      supports |= node instanceof RDXSpineAction spineAction && spineAction.getDefinition().getJointspaceOnly();
      return supports;
   }

   public RDXBehaviorTreeNode<?, ?> mirrorNode(RDXBehaviorTreeNode<?, ?> node)
   {
      ObjectNode jsonNode = nodeToJSON(node);

      RobotSide originalSide = RobotSide.getSideFromString(jsonNode.get("side").asText());
      RobotSide mirrorSide = originalSide.getOppositeSide();
      jsonNode.put("side", mirrorSide.getLowerCaseName());

      // Mirror
      if (node instanceof RDXArmAction && jsonNode.get("preset").asText().equals(ArmActionDefinition.CUSTOM_ANGLES_NAME))
      {
         ArmJointName[] armJointNames = behaviorTree.getRootNode().getSyncedRobot().getRobotModel().getJointMap().getArmJointNames();
         for (int i = 0; i < armJointNames.length; i++)
            if (!armJointNames[i].name().contains("PITCH"))
               jsonNode.put("j" + i + "Degrees", -jsonNode.get("j" + i + "Degrees").asDouble());
      }
      else if (node instanceof RDXNeckAction)
         jsonNode.put("yawInDegrees", -jsonNode.get("yawInDegrees").asDouble());
      else if (node instanceof RDXSpineAction)
      {
         SpineJointName[] spineJointNames = behaviorTree.getRootNode().getSyncedRobot().getRobotModel().getJointMap().getSpineJointNames();
         for (int i = 0; i < spineJointNames.length; i++)
            if (!spineJointNames[i].name().contains("PITCH"))
               jsonNode.put("j" + i, -jsonNode.get("j" + i).asDouble());
      }

      return jsonToNode(jsonNode);
   }

   public RDXLeafNode<?, ?> duplicateLeaf(RDXLeafNode<?, ?> leaf)
   {
      return (RDXLeafNode<?, ?>) jsonToNode(nodeToJSON(leaf));
   }

   private RDXBehaviorTreeNode<?, ?> jsonToNode(ObjectNode jsonNode)
   {
      RDXBehaviorTreeNode<?, ?> newNode = behaviorTree.getNodeBuilder().createNode(getClassFromTypeName(jsonNode.get("type").asText()),
                                                                                   behaviorTree.getAndIncrementNextID(),
                                                                                   behaviorTree.getRootNode());
      newNode.getDefinition().loadFromFile(jsonNode);
      return newNode;
   }

   private ObjectNode nodeToJSON(RDXBehaviorTreeNode<?, ?> node)
   {
      ObjectMapper mapper = new ObjectMapper();
      ObjectNode saveNode = mapper.createObjectNode();
      node.getDefinition().saveToFile(saveNode); // exploit the JSON methods to copy the definition
      return (ObjectNode) ExceptionTools.handle(() -> mapper.readTree(mapper.writeValueAsString(saveNode)), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }
}
