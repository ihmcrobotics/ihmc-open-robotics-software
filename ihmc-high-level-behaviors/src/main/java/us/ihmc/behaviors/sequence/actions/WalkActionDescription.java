package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionDescriptionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.FrameBasedBehaviorActionDescription;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

public class WalkActionDescription extends FrameBasedBehaviorActionDescription
{
   private String description = "Walk";
   private double swingDuration = 1.2;
   private double transferDuration = 0.8;
   private final SideDependentList<RigidBodyTransform> goalFootstepToParentTransforms = new SideDependentList<>(() -> new RigidBodyTransform());

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      JSONTools.toJSON(jsonNode, getTransformToParent());
      for (RobotSide side : RobotSide.values)
      {
         ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toJSON(goalFootNode, goalFootstepToParentTransforms.get(side));
      }
      jsonNode.put("swingDuration", swingDuration);
      jsonNode.put("transferDuration", transferDuration);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      getConditionalReferenceFrame().setParentFrameName(jsonNode.get("parentFrame").textValue());
      System.out.println(getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      JSONTools.toEuclid(jsonNode, getTransformToParent());
      for (RobotSide side : RobotSide.values)
      {
         JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toEuclid(goalFootNode, goalFootstepToParentTransforms.get(side));
      }
      swingDuration = jsonNode.get("swingDuration").asDouble();
      transferDuration = jsonNode.get("transferDuration").asDouble();
   }

   public void toMessage(WalkActionDescriptionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      MessageTools.toMessage(getTransformToParent(), message.getTransformToParent());
      MessageTools.toMessage(goalFootstepToParentTransforms.get(RobotSide.LEFT), message.getLeftGoalFootTransformToGizmo());
      MessageTools.toMessage(goalFootstepToParentTransforms.get(RobotSide.RIGHT), message.getRightGoalFootTransformToGizmo());
      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);
   }

   public void fromMessage(WalkActionDescriptionMessage message)
   {
      getConditionalReferenceFrame().setParentFrameName(message.getParentFrame().getString(0));
      MessageTools.toEuclid(message.getTransformToParent(), getTransformToParent());
      MessageTools.toEuclid(message.getLeftGoalFootTransformToGizmo(), goalFootstepToParentTransforms.get(RobotSide.LEFT));
      MessageTools.toEuclid(message.getRightGoalFootTransformToGizmo(), goalFootstepToParentTransforms.get(RobotSide.RIGHT));
      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();
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

   public SideDependentList<RigidBodyTransform> getGoalFootstepToParentTransforms()
   {
      return goalFootstepToParentTransforms;
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
}
