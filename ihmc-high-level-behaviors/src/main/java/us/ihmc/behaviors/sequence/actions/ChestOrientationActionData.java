package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ChestOrientationActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.JSONTools;

import java.util.function.Consumer;

public class ChestOrientationActionData implements BehaviorActionData
{
   private String description = "Chest orientation";
   private final YawPitchRoll yawPitchRoll = new YawPitchRoll();
   private double trajectoryDuration = 4.0;
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
      jsonNode.put("parentFrame", modifiableReferenceFrame.getReferenceFrame().getParent().getName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("yaw", yawPitchRoll.getYaw());
      jsonNode.put("pitch", yawPitchRoll.getPitch());
      jsonNode.put("roll", yawPitchRoll.getRoll());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      modifiableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByName(jsonNode.get("parentFrame").asText()).get());
      modifiableReferenceFrame.update(transformToParent -> JSONTools.toEuclid(jsonNode, transformToParent));
   }

   public void toMessage(ChestOrientationActionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getParentReferenceFrame().getName());
      MessageTools.toMessage(modifiableReferenceFrame.getTransformToParent(), message.getTransformToParent());
      message.setTrajectoryDuration(trajectoryDuration);
   }

   public void fromMessage(ChestOrientationActionMessage message)
   {
      modifiableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByName(message.getParentFrame().getString(0)).get());
      modifiableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
      trajectoryDuration = message.getTrajectoryDuration();
      setYawPitchRoll();
   }

   public YawPitchRoll getYawPitchRoll()
   {
      return yawPitchRoll;
   }

   public void setYaw(double yaw)
   {
      yawPitchRoll.setYaw(yaw);
      getTransformToParent().getRotation().setYawPitchRoll(yaw, yawPitchRoll.getPitch(), yawPitchRoll.getRoll());
   }

   public void setPitch(double pitch)
   {
      yawPitchRoll.setPitch(pitch);
      getTransformToParent().getRotation().setYawPitchRoll(yawPitchRoll.getYaw(), pitch, yawPitchRoll.getRoll());
   }

   public void setRoll(double roll)
   {
      yawPitchRoll.setRoll(roll);
      getTransformToParent().getRotation().setYawPitchRoll(yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), roll);
   }

   public void setRotation(RotationMatrixBasics rotation)
   {
      getTransformToParent().getRotation().set(rotation);
      setYawPitchRoll();
   }

   private void setYawPitchRoll()
   {
      RotationMatrixBasics rotation = getTransformToParent().getRotation();
      yawPitchRoll.setYawPitchRoll(rotation.getYaw(), rotation.getPitch(), rotation.getRoll());
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
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
