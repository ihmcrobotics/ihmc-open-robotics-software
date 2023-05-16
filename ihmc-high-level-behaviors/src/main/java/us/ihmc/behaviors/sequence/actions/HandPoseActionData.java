package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONTools;

public class HandPoseActionData implements BehaviorActionData
{
   private RobotSide side = RobotSide.LEFT;
   private String parentFrameName = "";
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();
   private double trajectoryDuration = 4.0;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("parentFrame", parentFrameName);
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      JSONTools.toJSON(jsonNode, transformToParent);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      parentFrameName = jsonNode.get("parentFrame").asText();
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      JSONTools.toEuclid(jsonNode, transformToParent);
   }

   public void toMessage(HandPoseActionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(parentFrameName);
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      MessageTools.toMessage(transformToParent, message.getTransformToParent());
   }

   public void fromMessage(HandPoseActionMessage message)
   {
      parentFrameName = message.getParentFrame().getString(0);
      side = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
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

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
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
