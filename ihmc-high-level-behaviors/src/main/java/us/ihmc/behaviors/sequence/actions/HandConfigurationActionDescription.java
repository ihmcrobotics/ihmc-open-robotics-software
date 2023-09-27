package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDescription;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandConfigurationActionDescription implements BehaviorActionDescription
{
   private String description = "Hand configuration";
   private RobotSide side = RobotSide.LEFT;
   private int handConfigurationIndex = 6;
   private boolean executeWitNextAction = false;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("grip", HandConfiguration.values[handConfigurationIndex].name());
      jsonNode.put("executeWithNextAction", executeWitNextAction);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      handConfigurationIndex = HandConfiguration.valueOf(jsonNode.get("grip").asText()).ordinal();
      executeWitNextAction = jsonNode.get("executeWithNextAction").asBoolean();
   }

   public void toMessage(HandConfigurationActionDescriptionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.setGrip(handConfigurationIndex);
      message.setExecuteWithNextAction(executeWitNextAction);
   }

   public void fromMessage(HandConfigurationActionDescriptionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      handConfigurationIndex = (int) message.getGrip();
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
