package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;

public class HandWrenchActionDefinition extends ActionNodeDefinition implements SidedObject
{
   private RobotSide side = RobotSide.LEFT;
   private double trajectoryDuration = 1000.0;
   private double force = 20.0;

   public HandWrenchActionDefinition()
   {
      setDescription("Hand wrench");
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("force", force);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      force = jsonNode.get("force").asDouble();
   }

   public void toMessage(HandWrenchActionDefinitionMessage message)
   {
      super.toMessage(message.getActionDefinition());

      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      message.setForce(force);
   }

   public void fromMessage(HandWrenchActionDefinitionMessage message)
   {
      super.fromMessage(message.getActionDefinition());

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

   @Override
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
}
