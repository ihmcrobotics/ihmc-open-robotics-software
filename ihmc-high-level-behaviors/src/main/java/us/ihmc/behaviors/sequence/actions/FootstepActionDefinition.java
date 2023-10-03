package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONTools;

public class FootstepActionDefinition implements BehaviorActionDefinition<FootstepActionDefinitionMessage>
{
   private String description = "Footstep";
   private RobotSide side = RobotSide.LEFT;
   private final RigidBodyTransform soleToPlanFrameTransform = new RigidBodyTransform();

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

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("side", side.getLowerCaseName());
      JSONTools.toJSON(jsonNode, soleToPlanFrameTransform);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      JSONTools.toEuclid(jsonNode, soleToPlanFrameTransform);
   }

   @Override
   public void toMessage(FootstepActionDefinitionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.getSolePose().set(soleToPlanFrameTransform);
   }

   @Override
   public void fromMessage(FootstepActionDefinitionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      soleToPlanFrameTransform.set(message.getSolePose());
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
