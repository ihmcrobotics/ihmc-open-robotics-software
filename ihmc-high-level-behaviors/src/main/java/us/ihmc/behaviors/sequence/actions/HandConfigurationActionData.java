package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandConfigurationActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandConfigurationActionData implements BehaviorActionData
{
   private RobotSide side = RobotSide.LEFT;
   private int handConfigurationIndex = 6;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("grip", HandConfiguration.values[handConfigurationIndex].name());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      handConfigurationIndex = HandConfiguration.valueOf(jsonNode.get("grip").asText()).ordinal();
   }

   public void toMessage(HandConfigurationActionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.setGrip(handConfigurationIndex);
   }

   public void fromMessage(HandConfigurationActionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      handConfigurationIndex = (int) message.getGrip();
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
}
