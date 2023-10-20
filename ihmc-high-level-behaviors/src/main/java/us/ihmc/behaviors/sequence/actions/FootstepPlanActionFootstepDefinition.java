package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;

public class FootstepPlanActionFootstepDefinition implements SidedObject
{
   private RobotSide side = RobotSide.LEFT;
   private final RigidBodyTransform soleToPlanFrameTransform = new RigidBodyTransform();

   @Override
   public RobotSide getSide()
   {
      return side;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
   }

   public RigidBodyTransform getSoleToPlanFrameTransform()
   {
      return soleToPlanFrameTransform;
   }

   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("side", side.getLowerCaseName());
      JSONTools.toJSON(jsonNode, soleToPlanFrameTransform);
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      JSONTools.toEuclid(jsonNode, soleToPlanFrameTransform);
   }

   public void toMessage(FootstepPlanActionFootstepDefinitionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.getSolePose().set(soleToPlanFrameTransform);
   }

   public void fromMessage(FootstepPlanActionFootstepDefinitionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      soleToPlanFrameTransform.set(message.getSolePose());
   }
}
