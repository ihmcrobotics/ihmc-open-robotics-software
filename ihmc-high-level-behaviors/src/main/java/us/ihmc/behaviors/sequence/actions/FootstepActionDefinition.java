package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONTools;

public class FootstepActionDefinition
{
   private RobotSide side = RobotSide.LEFT;
   private final Pose3D solePose = new Pose3D();

   public RobotSide getSide()
   {
      return side;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
   }

   public Pose3D getSolePose()
   {
      return solePose;
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("side", side.getLowerCaseName());
      JSONTools.toJSON(jsonNode, solePose);
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      JSONTools.toEuclid(jsonNode, solePose);
   }

   public void toMessage(FootstepActionDefinitionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.getSolePose().set(solePose);
   }

   public void fromMessage(FootstepActionDefinitionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      solePose.set(message.getSolePose());
   }
}
