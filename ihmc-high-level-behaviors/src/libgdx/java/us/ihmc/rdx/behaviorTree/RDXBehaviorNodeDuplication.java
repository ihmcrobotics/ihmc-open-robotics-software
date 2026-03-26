package us.ihmc.rdx.behaviorTree;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.databind.node.TextNode;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.rdx.behaviorTree.actions.RDXArmAction;
import us.ihmc.rdx.behaviorTree.actions.RDXNeckAction;
import us.ihmc.rdx.behaviorTree.actions.RDXSceneAction;
import us.ihmc.rdx.behaviorTree.actions.RDXScrewPrimitiveAction;
import us.ihmc.rdx.behaviorTree.actions.RDXSpineAction;
import us.ihmc.rdx.behaviorTree.actions.RDXWalkAction;
import us.ihmc.rdx.behaviorTree.condition.RDXConditionNode;
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

      if (jsonNode.has("executeAfterAction"))
         jsonNode.put("executeAfterAction", StringUtils.replaceEach(jsonNode.get("executeAfterAction").asText(),
                                                                    new String[]{"Left", "Right", "left", "right", "LEFT", "RIGHT"},
                                                                    new String[]{"Right", "Left", "right", "left", "RIGHT", "LEFT"}));

      applyFrameInvariantMirroring(node, jsonNode);
      if (node instanceof RDXConditionNode)
      {
         String[] fieldNames = new String[] { "frameName", "frameNameA", "frameNameB" };
         for (String fieldName : fieldNames)
            if (jsonNode.get(fieldName) instanceof TextNode textNode)
            {
               if (textNode.asText().contains("Left"))
                  jsonNode.put(fieldName, textNode.asText().replace("Left", "Right"));
               else if (textNode.asText().contains("Right"))
                  jsonNode.put(fieldName, textNode.asText().replace("Right", "Left"));
            }
         if (jsonNode.get("shapeTransformToParent") instanceof ObjectNode shapeTransformToParent)
         {
            shapeTransformToParent.put("y", -shapeTransformToParent.get("y").asDouble());
            shapeTransformToParent.put("rollInDegrees", -shapeTransformToParent.get("rollInDegrees").asDouble());
            shapeTransformToParent.put("yawInDegrees", -shapeTransformToParent.get("yawInDegrees").asDouble());
         }
      }
      else if (node instanceof RDXSceneAction)
      {
         if (jsonNode.get("nominalObjectPose") instanceof ObjectNode nominalObjectPose)
         {
            nominalObjectPose.put("y", -nominalObjectPose.get("y").asDouble());
            nominalObjectPose.put("rollInDegrees", -nominalObjectPose.get("rollInDegrees").asDouble());
            nominalObjectPose.put("yawInDegrees", -nominalObjectPose.get("yawInDegrees").asDouble());
         }
      }
      else if (node instanceof RDXArmAction armAction && !armAction.getDefinition().getUsePredefinedJointAngles())
      {
         jsonNode.put("y", -jsonNode.get("y").asDouble()); // TODO: This is specific to the door lever object pose
         jsonNode.put("rollInDegrees", -jsonNode.get("rollInDegrees").asDouble());
         jsonNode.put("yawInDegrees", -jsonNode.get("yawInDegrees").asDouble());
      }
      else if (node instanceof RDXScrewPrimitiveAction)
      {
         if (jsonNode.get("screwAxisPose") instanceof ObjectNode screwAxisPose)
         {
            screwAxisPose.put("y", -screwAxisPose.get("y").asDouble());
            screwAxisPose.put("rollInDegrees", -screwAxisPose.get("rollInDegrees").asDouble());
            screwAxisPose.put("yawInDegrees", -screwAxisPose.get("yawInDegrees").asDouble());
         }
         jsonNode.put("rotation", -jsonNode.get("rotation").asDouble());
      }
      else if (node instanceof RDXWalkAction)
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
               JsonNode leftX = leftGoalFootToGoal.get("x"); // TODO: This is specific to the door panel object pose
               JsonNode leftY = leftGoalFootToGoal.get("y");
               JsonNode rightX = rightGoalFootToGoal.get("x");
               JsonNode rightY = rightGoalFootToGoal.get("y");
               leftGoalFootToGoal.put("x", rightX.asDouble());
               rightGoalFootToGoal.put("x", leftX.asDouble());
               leftGoalFootToGoal.put("y", -rightY.asDouble());
               rightGoalFootToGoal.put("y", -leftY.asDouble());
            }
         }
         else if (jsonNode.get("footsteps") instanceof ArrayNode footsteps)
         {
            for (int i = 0; i < footsteps.size(); i++)
            {
               if (footsteps.get(i) instanceof ObjectNode footstep)
               {
                  footstep.put("side", footstep.get("side").asText().contains("left") ? "right" : "left");
                  footstep.put("y", -footstep.get("y").asDouble());
                  footstep.put("rollInDegrees", -footstep.get("rollInDegrees").asDouble());
                  footstep.put("yawInDegrees", -footstep.get("yawInDegrees").asDouble());
               }
            }
         }
      }

      return jsonToNode(jsonNode);
   }

   public RDXBehaviorTreeNode<?, ?> mirrorNode(RDXBehaviorTreeNode<?, ?> node)
   {
      ObjectNode jsonNode = nodeToJSONMirror(node);
      applyFrameInvariantMirroring(node, jsonNode);
      return jsonToNode(jsonNode);
   }

   private void applyFrameInvariantMirroring(RDXBehaviorTreeNode<?, ?> node, ObjectNode jsonNode)
   {
      if (node instanceof RDXArmAction armAction
          && armAction.getDefinition().getUsePredefinedJointAngles()
          && jsonNode.get("preset").asText().equals(ArmActionDefinition.CUSTOM_ANGLES_NAME))
      {
         ArmJointName[] armJointNames = behaviorTree.getRootNode().getSyncedRobot().getRobotModel().getJointMap().getArmJointNames();
         for (int i = 0; i < armJointNames.length; i++)
            if (!armJointNames[i].name().contains("PITCH"))
               jsonNode.put("j" + i + "Degrees", -jsonNode.get("j" + i + "Degrees").asDouble());
      }
      else if (node instanceof RDXNeckAction)
         jsonNode.put("yawInDegrees", -jsonNode.get("yawInDegrees").asDouble());
      else if (node instanceof RDXSpineAction spineAction && spineAction.getDefinition().getJointspaceOnly())
      {
         SpineJointName[] spineJointNames = behaviorTree.getRootNode().getSyncedRobot().getRobotModel().getJointMap().getSpineJointNames();
         for (int i = 0; i < spineJointNames.length; i++)
            if (!spineJointNames[i].name().contains("PITCH"))
               jsonNode.put("j" + i, -jsonNode.get("j" + i).asDouble());
      }
   }

   private ObjectNode nodeToJSONMirror(RDXBehaviorTreeNode<?, ?> node)
   {
      ObjectNode jsonNode = nodeToJSON(node);

      jsonNode.put("name", StringUtils.replaceEach(jsonNode.get("name").asText(),
                                                   new String[]{"Left", "Right", "left", "right", "LEFT", "RIGHT"},
                                                   new String[]{"Right", "Left", "right", "left", "RIGHT", "LEFT"}));

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

   public RDXBehaviorTreeNode<?, ?> mirrorSubtree(RDXBehaviorTreeNode<?, ?> node, RDXBehaviorTreeNode<?, ?> duplicateParent)
   {
      RDXBehaviorTreeNode<?, ?> duplicate = mirrorNode(node);
      if (duplicateParent != null)
         topologyOperationQueue.queueAppendChildModify(duplicateParent, duplicate);
      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         mirrorSubtree(child, duplicate);
      return duplicate;
   }

   public RDXBehaviorTreeNode<?, ?> mirrorSubtreeDoorSpecific(RDXBehaviorTreeNode<?, ?> node, RDXBehaviorTreeNode<?, ?> duplicateParent)
   {
      RDXBehaviorTreeNode<?, ?> duplicate = mirrorNodeDoorSpecific(node);
      if (duplicateParent != null)
         topologyOperationQueue.queueAppendChildModify(duplicateParent, duplicate);
      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         mirrorSubtreeDoorSpecific(child, duplicate);
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
