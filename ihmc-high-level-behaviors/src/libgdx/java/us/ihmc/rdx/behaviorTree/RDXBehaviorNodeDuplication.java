package us.ihmc.rdx.behaviorTree;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.rdx.behaviorTree.actions.RDXAbilityHandAction;
import us.ihmc.rdx.behaviorTree.actions.RDXArmAction;
import us.ihmc.rdx.behaviorTree.actions.RDXEZGripperAction;
import us.ihmc.rdx.behaviorTree.actions.RDXNeckAction;
import us.ihmc.rdx.behaviorTree.actions.RDXSpineAction;
import us.ihmc.rdx.behaviorTree.actions.RDXWalkAction;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry.getClassFromTypeName;

public class RDXBehaviorNodeDuplication
{
   private final RDXBehaviorTree behaviorTree;
   private final BehaviorTreeTopologyOperationQueue<RDXBehaviorTreeNode<?, ?>> topologyOperationQueue;

   public RDXBehaviorNodeDuplication(RDXBehaviorTree behaviorTree)
   {
      this.behaviorTree = behaviorTree;
      topologyOperationQueue = behaviorTree.getTopologyChangeQueue();
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

   public boolean supportsDoorSpecificMirroring(RDXBehaviorTreeNode<?, ?> node)
   {
      boolean supports = false;
      supports |= node instanceof RDXWalkAction;
      supports |= node instanceof RDXArmAction;
      return supports;
   }

   public RDXBehaviorTreeNode<?, ?> mirrorNodeDoorSpecific(RDXBehaviorTreeNode<?, ?> node)
   {
      ObjectNode jsonNode = nodeToJSONMirror(node);

      if (node instanceof RDXWalkAction)
      {
         if (jsonNode.has("goalStancePoint"))
         {
            if (jsonNode.get("goalStancePoint") instanceof ObjectNode goalStancePoint)
               goalStancePoint.put("y", -goalStancePoint.get("y").asDouble());
            if (jsonNode.get("goalFocalPoint") instanceof ObjectNode goalFocalPoint)
               goalFocalPoint.put("y", -goalFocalPoint.get("y").asDouble());
            if (jsonNode.get("leftGoalFootToGoal") instanceof ObjectNode leftGoalFootToGoal
             && jsonNode.get("rightGoalFootToGoal") instanceof ObjectNode rightGoalFootToGoal)
            {
               JsonNode leftX = leftGoalFootToGoal.get("x");
               JsonNode leftY = leftGoalFootToGoal.get("y");
               JsonNode rightX = rightGoalFootToGoal.get("x");
               JsonNode rightY = rightGoalFootToGoal.get("y");
               leftGoalFootToGoal.put("x", rightX.asDouble());
               rightGoalFootToGoal.put("x", leftX.asDouble());
               leftGoalFootToGoal.put("y", -rightY.asDouble());
               rightGoalFootToGoal.put("y", -leftY.asDouble());
            }
         }
         else if (jsonNode.has("footsteps"))
         {

         }
      }
      else if (node instanceof RDXArmAction)
      {

      }

      return jsonToNode(jsonNode);
   }

   public RDXBehaviorTreeNode<?, ?> mirrorNode(RDXBehaviorTreeNode<?, ?> node)
   {
      ObjectNode jsonNode = nodeToJSONMirror(node);

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

   private ObjectNode nodeToJSONMirror(RDXBehaviorTreeNode<?, ?> node)
   {
      ObjectNode jsonNode = nodeToJSON(node);

      String originalName = jsonNode.get("name").asText();
      jsonNode.put("name", originalName.replace("Left", "Right").replace("left", "right").replace("LEFT", "RIGHT"));

      if (jsonNode.has("side"))
      {
         RobotSide originalSide = RobotSide.getSideFromString(jsonNode.get("side").asText());
         RobotSide mirrorSide = originalSide.getOppositeSide();
         jsonNode.put("side", mirrorSide.getLowerCaseName());
      }

      return jsonNode;
   }

   public RDXBehaviorTreeNode<?, ?> duplicateNode(RDXBehaviorTreeNode<?, ?> node)
   {
      return jsonToNode(nodeToJSON(node));
   }

   public RDXBehaviorTreeNode<?, ?> duplicateSubtree(RDXBehaviorTreeNode<?, ?> node, RDXBehaviorTreeNode<?, ?> duplicateParent)
   {
      RDXBehaviorTreeNode<?, ?> duplicate = duplicateNode(node);

      if (duplicateParent != null)
         topologyOperationQueue.queueAppendChildModify(duplicateParent, duplicate);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         duplicateSubtree(child, duplicate);

      return duplicate;
   }

   private RDXBehaviorTreeNode<?, ?> jsonToNode(ObjectNode jsonNode)
   {
      RDXBehaviorTreeNode<?, ?> newNode = behaviorTree.getNodeBuilder().createNode(getClassFromTypeName(jsonNode.get("type").asText()),
                                                                                   behaviorTree.getAndIncrementNextID(),
                                                                                   behaviorTree.getRootNode());
      newNode.getDefinition().modify();
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
