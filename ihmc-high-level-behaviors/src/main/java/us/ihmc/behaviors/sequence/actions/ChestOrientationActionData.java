package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ChestOrientationActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;

import java.util.function.Consumer;

public class ChestOrientationActionData implements BehaviorActionData
{
   private String description = "Chest orientation";
   private final YawPitchRoll yawPitchRoll = new YawPitchRoll();
   private double trajectoryDuration = 4.0;
   private final ModifiableReferenceFrame modifiableReferenceFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
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
      yawPitchRoll.setYaw(jsonNode.get("yaw").asDouble());
      yawPitchRoll.setPitch(jsonNode.get("pitch").asDouble());
      yawPitchRoll.setRoll(jsonNode.get("roll").asDouble());
   }

   public void toMessage(ChestOrientationActionMessage message)
   {
      message.setTrajectoryDuration(trajectoryDuration);
      message.getOrientation().setYaw(yawPitchRoll.getYaw());
      message.getOrientation().setPitch(yawPitchRoll.getPitch());
      message.getOrientation().setRoll(yawPitchRoll.getRoll());
   }

   public void fromMessage(ChestOrientationActionMessage message)
   {
      trajectoryDuration = message.getTrajectoryDuration();
      yawPitchRoll.setYaw(message.getOrientation().getYaw());
      yawPitchRoll.setPitch(message.getOrientation().getPitch());
      yawPitchRoll.setRoll(message.getOrientation().getRoll());
   }

   public YawPitchRoll getYawPitchRoll()
   {
      return yawPitchRoll;
   }

   public void setYawPitchRoll(RigidBodyTransform transformToParent)
   {
      RotationMatrixBasics rotationToParent = transformToParent.getRotation();
      this.yawPitchRoll.setYawPitchRoll(rotationToParent.getYaw(), rotationToParent.getPitch(), rotationToParent.getRoll());
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
