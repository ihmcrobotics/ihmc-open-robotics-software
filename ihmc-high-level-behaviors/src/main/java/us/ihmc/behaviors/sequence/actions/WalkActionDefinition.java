package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

public class WalkActionDefinition implements BehaviorActionDefinition
{
   private String description = "Walk";
   private double swingDuration = 1.2;
   private double transferDuration = 0.8;
   private String parentFrameName;
   private RigidBodyTransform transformToParent = new RigidBodyTransform();
   private final SideDependentList<RigidBodyTransform> goalFootstepToParentTransforms = new SideDependentList<>(() -> new RigidBodyTransform());

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("swingDuration", swingDuration);
      jsonNode.put("transferDuration", transferDuration);
      jsonNode.put("parentFrame", parentFrameName);
      JSONTools.toJSON(jsonNode, transformToParent);
      for (RobotSide side : RobotSide.values)
      {
         ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toJSON(goalFootNode, goalFootstepToParentTransforms.get(side));
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      swingDuration = jsonNode.get("swingDuration").asDouble();
      transferDuration = jsonNode.get("transferDuration").asDouble();
      parentFrameName = jsonNode.get("parentFrame").textValue();
      JSONTools.toEuclid(jsonNode, transformToParent);
      for (RobotSide side : RobotSide.values)
      {
         JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toEuclid(goalFootNode, goalFootstepToParentTransforms.get(side));
      }
   }

   public void toMessage(WalkActionDefinitionMessage message)
   {
      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(parentFrameName);
      MessageTools.toMessage(transformToParent, message.getTransformToParent());
      MessageTools.toMessage(goalFootstepToParentTransforms.get(RobotSide.LEFT), message.getLeftGoalFootTransformToGizmo());
      MessageTools.toMessage(goalFootstepToParentTransforms.get(RobotSide.RIGHT), message.getRightGoalFootTransformToGizmo());
   }

   public void fromMessage(WalkActionDefinitionMessage message)
   {
      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();
      parentFrameName = message.getParentFrame().getString(0);
      MessageTools.toEuclid(message.getTransformToParent(), transformToParent);
      MessageTools.toEuclid(message.getLeftGoalFootTransformToGizmo(), goalFootstepToParentTransforms.get(RobotSide.LEFT));
      MessageTools.toEuclid(message.getRightGoalFootTransformToGizmo(), goalFootstepToParentTransforms.get(RobotSide.RIGHT));
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

   public String getParentFrameName()
   {
      return parentFrameName;
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName = parentFrameName;
   }

   public RigidBodyTransform getTransformToParent()
   {
      return transformToParent;
   }

   public void setTransformToParent(RigidBodyTransform transformToParent)
   {
      this.transformToParent = transformToParent;
   }
}
