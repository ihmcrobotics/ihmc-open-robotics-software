package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

public class WalkActionDefinition implements BehaviorActionDefinition<WalkActionDefinitionMessage>
{
   private String description = "Walk";
   private double swingDuration = 1.2;
   private double transferDuration = 0.8;
   private String parentFrameName;
   private final RigidBodyTransform goalToParentTransform = new RigidBodyTransform();
   private final SideDependentList<RigidBodyTransform> goalFootstepToGoalTransforms = new SideDependentList<>(() -> new RigidBodyTransform());

   public WalkActionDefinition(FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      for (RobotSide side : RobotSide.values)
      {
         goalFootstepToGoalTransforms.get(side).getTranslation().addY(0.5 * side.negateIfRightSide(footstepPlannerParameters.getIdealFootstepWidth()));
      }
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("swingDuration", swingDuration);
      jsonNode.put("transferDuration", transferDuration);
      jsonNode.put("parentFrame", parentFrameName);
      JSONTools.toJSON(jsonNode, goalToParentTransform);
      for (RobotSide side : RobotSide.values)
      {
         ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toJSON(goalFootNode, goalFootstepToGoalTransforms.get(side));
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      swingDuration = jsonNode.get("swingDuration").asDouble();
      transferDuration = jsonNode.get("transferDuration").asDouble();
      parentFrameName = jsonNode.get("parentFrame").textValue();
      JSONTools.toEuclid(jsonNode, goalToParentTransform);
      for (RobotSide side : RobotSide.values)
      {
         JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toEuclid(goalFootNode, goalFootstepToGoalTransforms.get(side));
      }
   }

   @Override
   public void toMessage(WalkActionDefinitionMessage message)
   {
      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(parentFrameName);
      MessageTools.toMessage(goalToParentTransform, message.getTransformToParent());
      MessageTools.toMessage(goalFootstepToGoalTransforms.get(RobotSide.LEFT), message.getLeftGoalFootTransformToGizmo());
      MessageTools.toMessage(goalFootstepToGoalTransforms.get(RobotSide.RIGHT), message.getRightGoalFootTransformToGizmo());
   }

   @Override
   public void fromMessage(WalkActionDefinitionMessage message)
   {
      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();
      parentFrameName = message.getParentFrame().getString(0);
      MessageTools.toEuclid(message.getTransformToParent(), goalToParentTransform);
      MessageTools.toEuclid(message.getLeftGoalFootTransformToGizmo(), goalFootstepToGoalTransforms.get(RobotSide.LEFT));
      MessageTools.toEuclid(message.getRightGoalFootTransformToGizmo(), goalFootstepToGoalTransforms.get(RobotSide.RIGHT));
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   public double getTransferDuration()
   {
      return transferDuration;
   }

   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration = transferDuration;
   }

   public SideDependentList<RigidBodyTransform> getGoalFootstepToGoalTransforms()
   {
      return goalFootstepToGoalTransforms;
   }

   @Override
   public String getDescription()
   {
      return description;
   }

   @Override
   public void setDescription(String description)
   {
      this.description = description;
   }

   public String getParentFrameName()
   {
      return parentFrameName;
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName = parentFrameName;
   }

   public RigidBodyTransform getGoalToParentTransform()
   {
      return goalToParentTransform;
   }
}
