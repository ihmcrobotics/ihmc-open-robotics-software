package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandWrenchActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandWrenchActionData implements BehaviorActionData
{
   private String description = "Hand wrench";
   private RobotSide side = RobotSide.LEFT;
   private double trajectoryDuration = 1000.0;
   private double force = 20.0;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("force", force);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      force = jsonNode.get("force").asDouble();
   }

   public void toMessage(HandWrenchActionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      message.setForce(force);
   }

   public void fromMessage(HandWrenchActionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
      force = message.getForce();
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public double getForce()
   {
      return force;
   }

   public void setForce(double force)
   {
      this.force = force;
   }

   public RobotSide getSide()
   {
      return side;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
   }

   @Override
   public void setDescription(String description)
   {
      this.description = description;
   }

   @Override
   public String getDescription()
   {
      return description;
   }
}
