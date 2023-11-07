package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.robotics.robotSide.RobotSide;

public class SakeHandCommandActionDefinition extends ActionNodeDefinition
{
   private RobotSide side = RobotSide.LEFT;
   private int handConfigurationIndex = SakeHandCommandOption.GOTO.ordinal();
   private double goalPosition = 1.0;  // default to open
   private double goalTorque = 0.0;    // default to none
   private boolean executeWitNextAction = false;

   public SakeHandCommandActionDefinition(CRDTInfo crdtInfo)
   {
      super(crdtInfo);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("configuration", SakeHandCommandOption.values[handConfigurationIndex].name());
      jsonNode.put("position", goalPosition);
      jsonNode.put("torque", goalTorque);
      jsonNode.put("executeWithNextAction", executeWitNextAction);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      handConfigurationIndex = SakeHandCommandOption.valueOf(jsonNode.get("configuration").asText()).ordinal();
      goalPosition = jsonNode.get("position").asDouble();
      goalTorque = jsonNode.get("torque").asDouble();
      executeWitNextAction = jsonNode.get("executeWithNextAction").asBoolean();
   }

   public void toMessage(SakeHandCommandActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toByte());
      message.setConfiguration(SakeHandCommandOption.values[handConfigurationIndex].getCommandNumber());
      message.setPositionRatio(goalPosition);
      message.setTorqueRatio(goalTorque);
      message.setExecuteWithNextAction(executeWitNextAction);
   }

   public void fromMessage(SakeHandCommandActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

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
}
