package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONTools;

import java.util.function.Consumer;

public class FootstepActionData implements BehaviorActionData
{
   private String description = "Footstep";
   private RobotSide side = RobotSide.LEFT;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ModifiableReferenceFrame modifiableReferenceFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());

   @Override
   public void setReferenceFrameLibrary(ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("parentFrame", modifiableReferenceFrame.getReferenceFrame().getParent().getName());
      JSONTools.toJSON(jsonNode, modifiableReferenceFrame.getTransformToParent());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      modifiableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByName(jsonNode.get("parentFrame").asText()).get());
      modifiableReferenceFrame.update(transformToParent -> JSONTools.toEuclid(jsonNode, transformToParent));
   }

   public void toMessage(FootstepActionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getParentReferenceFrame().getName());
      MessageTools.toMessage(modifiableReferenceFrame.getTransformToParent(), message.getTransformToParent());
   }

   public void fromMessage(FootstepActionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      modifiableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByName(message.getParentFrame().getString(0)).get());
      modifiableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
   }

   public ReferenceFrame getParentReferenceFrame()
   {
      return modifiableReferenceFrame.getReferenceFrame().getParent();
   }

   public ReferenceFrame getReferenceFrame()
   {
      return modifiableReferenceFrame.getReferenceFrame();
   }

   public void changeParentFrameWithoutMoving(ReferenceFrame parentFrame)
   {
      modifiableReferenceFrame.changeParentFrameWithoutMoving(parentFrame);
   }

   public void changeParentFrame(ReferenceFrame parentFrame)
   {
      modifiableReferenceFrame.changeParentFrame(parentFrame);
   }

   public void setTransformToParent(Consumer<RigidBodyTransform> transformToParentConsumer)
   {
      modifiableReferenceFrame.update(transformToParentConsumer);
   }

   public RigidBodyTransform getTransformToParent()
   {
      return modifiableReferenceFrame.getTransformToParent();
   }

   public RobotSide getSide()
   {
      return side;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
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
