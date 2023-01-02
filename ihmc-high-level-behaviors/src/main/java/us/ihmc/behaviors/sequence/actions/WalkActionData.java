package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

public class WalkActionData implements BehaviorActionData
{
   private String parentFrameName = "";
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();
   private final SideDependentList<RigidBodyTransform> goalFootstepToGizmos = new SideDependentList<>(() -> new RigidBodyTransform());
   private double swingDuration = 1.2;
   private double transferDuration = 0.8;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("parentFrame", parentFrameName);
      JSONTools.toJSON(jsonNode, transformToParent);
      for (RobotSide side : RobotSide.values)
      {
         ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toJSON(goalFootNode, goalFootstepToGizmos.get(side));
      }
      jsonNode.put("swingDuration", swingDuration);
      jsonNode.put("transferDuration", transferDuration);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      parentFrameName = jsonNode.get("parentFrame").asText();
      JSONTools.toEuclid(jsonNode, transformToParent);
      for (RobotSide side : RobotSide.values)
      {
         JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toEuclid(goalFootNode, goalFootstepToGizmos.get(side));
      }
      swingDuration = jsonNode.get("swingDuration").asDouble();
      transferDuration = jsonNode.get("transferDuration").asDouble();
   }

   public void toMessage(WalkActionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(parentFrameName);
      MessageTools.toMessage(transformToParent, message.getTransformToParent());
      MessageTools.toMessage(goalFootstepToGizmos.get(RobotSide.LEFT), message.getLeftGoalFootTransformToGizmo());
      MessageTools.toMessage(goalFootstepToGizmos.get(RobotSide.RIGHT), message.getRightGoalFootTransformToGizmo());
      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);
   }

   public void fromMessage(WalkActionMessage message)
   {
      parentFrameName = message.getParentFrame().toString();
      MessageTools.toEuclid(message.getTransformToParent(), transformToParent);
      MessageTools.toEuclid(message.getLeftGoalFootTransformToGizmo(), goalFootstepToGizmos.get(RobotSide.LEFT));
      MessageTools.toEuclid(message.getRightGoalFootTransformToGizmo(), goalFootstepToGizmos.get(RobotSide.RIGHT));
      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();
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

   public SideDependentList<RigidBodyTransform> getGoalFootstepToGizmos()
   {
      return goalFootstepToGizmos;
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
}
