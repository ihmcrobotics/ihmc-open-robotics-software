package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONTools;

public class FootstepActionData implements BehaviorActionData
{
   private RobotSide side = RobotSide.LEFT;
   private String parentFrameName = "";
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("parentFrame", parentFrameName);
      JSONTools.toJSON(jsonNode, transformToParent);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      parentFrameName = jsonNode.get("parentFrame").asText();
      JSONTools.toEuclid(jsonNode, transformToParent);
   }

   public void toMessage(FootstepActionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(parentFrameName);
      MessageTools.toMessage(transformToParent, message.getTransformToParent());
   }

   public void fromMessage(FootstepActionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      parentFrameName = message.getParentFrame().getString(0);
      MessageTools.toEuclid(message.getTransformToParent(), transformToParent);
   }

   public RobotSide getSide()
   {
      return side;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
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
}
