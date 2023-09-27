package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.avatar.sakeGripper.SakeHandParameters.*;

public class SakeHandCommandActionData implements BehaviorActionData
{
   private String description = "Hand configuration";
   private RobotSide side = RobotSide.LEFT;
   private int handConfigurationIndex = SakeCommandOption.GOTO.ordinal();
   private double goalPosition = 1.0;  // default to open
   private double goalTorque = 0.0;    // default to none
   private boolean executeWitNextAction = false;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("configuration", SakeCommandOption.values[handConfigurationIndex].name());
      jsonNode.put("position", goalPosition);
      jsonNode.put("torque", goalTorque);
      jsonNode.put("executeWithNextAction", executeWitNextAction);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      handConfigurationIndex = SakeCommandOption.valueOf(jsonNode.get("configuration").asText()).ordinal();
      goalPosition = jsonNode.get("position").asDouble();
      goalTorque = jsonNode.get("torque").asDouble();
      executeWitNextAction = jsonNode.get("executeWithNextAction").asBoolean();
   }

   public void toMessage(SakeHandCommandActionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.setConfiguration(SakeCommandOption.values[handConfigurationIndex].getCommandNumber());
      message.setPositionRatio(goalPosition);
      message.setTorqueRatio(goalTorque);
      message.setExecuteWithNextAction(executeWitNextAction);
   }

   public void fromMessage(SakeHandCommandActionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      handConfigurationIndex = (int) message.getConfiguration();
      goalPosition = message.getPositionRatio();
      goalTorque = message.getTorqueRatio();
      executeWitNextAction = message.getExecuteWithNextAction();
   }

   public RobotSide getSide()
   {
      return side;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
   }

   public int getHandConfigurationIndex()
   {
      return handConfigurationIndex;
   }

   public void setHandConfigurationIndex(int handConfigurationIndex)
   {
      this.handConfigurationIndex = handConfigurationIndex;
   }

   public double getGoalPosition()
   {
      return goalPosition;
   }

   public double getGoalTorque()
   {
      return goalTorque;
   }

   public void setGoalPosition(double goalPosition)
   {
      this.goalPosition = goalPosition;
   }

   public void setGoalTorque(double goalTorque)
   {
      this.goalTorque = goalTorque;
   }

   public boolean getExecuteWithNextAction()
   {
      return executeWitNextAction;
   }

   public void setExecuteWithNextAction(boolean executeWitNextAction)
   {
      this.executeWitNextAction = executeWitNextAction;
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
