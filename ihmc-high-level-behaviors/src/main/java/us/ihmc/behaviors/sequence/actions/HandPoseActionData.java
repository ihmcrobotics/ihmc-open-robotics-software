package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionMessage;
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

public class HandPoseActionData implements BehaviorActionData
{
   private String description = "Hand pose";
   private RobotSide side = RobotSide.LEFT;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ModifiableReferenceFrame modifiableReferenceFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private double trajectoryDuration = 4.0;

   @Override
   public void setReferenceFrameLibrary(ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", modifiableReferenceFrame.getReferenceFrame().getParent().getName());
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      JSONTools.toJSON(jsonNode, modifiableReferenceFrame.getTransformToParent());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      modifiableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByName(jsonNode.get("parentFrame").asText()).get());
      modifiableReferenceFrame.update(transformToParent -> JSONTools.toEuclid(jsonNode, transformToParent));
   }

   public void toMessage(HandPoseActionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getParentReferenceFrame().getName());
      MessageTools.toMessage(modifiableReferenceFrame.getTransformToParent(), message.getTransformToParent());
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
   }

   public void fromMessage(HandPoseActionMessage message)
   {
      modifiableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByName(message.getParentFrame().getString(0)).get());
      modifiableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
      side = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
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

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
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
