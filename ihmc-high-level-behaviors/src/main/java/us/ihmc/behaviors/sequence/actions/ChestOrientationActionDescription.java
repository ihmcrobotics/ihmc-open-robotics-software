package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.BodyPartPoseActionDescriptionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.FrameBasedBehaviorActionData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.JSONTools;

public class ChestOrientationActionDescription implements FrameBasedBehaviorActionData
{
   private String description = "Chest orientation";
   private double trajectoryDuration = 4.0;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ModifiableReferenceFrame chestInteractableReferenceFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private boolean executeWitNextAction = false;
   private boolean holdPoseInWorldLater = false;

   @Override
   public String getDescription()
   {
      return description;
   }

   @Override
   public void setDescription(String description)
   {
      this.description = description;
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
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("parentFrame", getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      jsonNode.put("executeWithNextAction", executeWitNextAction);
      jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater);
      JSONTools.toJSON(jsonNode, chestInteractableReferenceFrame.getTransformToParent());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      getConditionalReferenceFrame().setParentFrameName(jsonNode.get("parentFrame").textValue());
      chestInteractableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(jsonNode.get("parentFrame").asText()));
      chestInteractableReferenceFrame.update(transformToParent -> JSONTools.toEuclid(jsonNode, transformToParent));
      executeWitNextAction = jsonNode.get("executeWithNextAction").asBoolean();
      holdPoseInWorldLater = jsonNode.get("holdPoseInWorldLater").asBoolean();
   }

   public void toMessage(BodyPartPoseActionDescriptionMessage message)
   {
      message.setTrajectoryDuration(trajectoryDuration);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      MessageTools.toMessage(getTransformToParent(), message.getTransformToParent());
      message.setExecuteWithNextAction(executeWitNextAction);
      message.setHoldPoseInWorld(holdPoseInWorldLater);
   }

   public void fromMessage(BodyPartPoseActionDescriptionMessage message)
   {
      getConditionalReferenceFrame().setParentFrameName(message.getParentFrame().getString(0));
      MessageTools.toEuclid(message.getTransformToParent(), getTransformToParent());
      chestInteractableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(message.getParentFrame().getString(0)));
      chestInteractableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
      trajectoryDuration = message.getTrajectoryDuration();
      executeWitNextAction = message.getExecuteWithNextAction();
      holdPoseInWorldLater = message.getHoldPoseInWorld();
   }

   public void setYaw(double yaw)
   {
      RotationMatrixBasics rotation = getTransformToParent().getRotation();
      getTransformToParent().getRotation().setYawPitchRoll(yaw, rotation.getPitch(), rotation.getRoll());
   }

   public void setPitch(double pitch)
   {
      RotationMatrixBasics rotation = getTransformToParent().getRotation();
      getTransformToParent().getRotation().setYawPitchRoll(rotation.getYaw(), pitch, rotation.getRoll());
   }

   public void setRoll(double roll)
   {
      RotationMatrixBasics rotation = getTransformToParent().getRotation();
      getTransformToParent().getRotation().setYawPitchRoll(rotation.getYaw(), rotation.getPitch(), roll);
   }

   public RotationMatrixBasics getRotation()
   {
      return getTransformToParent().getRotation();
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
   }

   public boolean getExecuteWithNextAction()
   {
      return executeWitNextAction;
   }

   public void setExecuteWithNextAction(boolean executeWitNextAction)
   {
      this.executeWitNextAction = executeWitNextAction;
   }

   public boolean getHoldPoseInWorldLater()
   {
      return holdPoseInWorldLater;
   }

   public void setHoldPoseInWorldLater(boolean holdPoseInWorldLater)
   {
      this.holdPoseInWorldLater = holdPoseInWorldLater;
   }

   public ReferenceFrame getParentFrame()
   {
      return chestInteractableReferenceFrame.getReferenceFrame().getParent();
   }

   public ReferenceFrame getReferenceFrame()
   {
      return chestInteractableReferenceFrame.getReferenceFrame();
   }

   public void changeParentFrameWithoutMoving(ReferenceFrame parentFrame)
   {
      chestInteractableReferenceFrame.changeParentFrameWithoutMoving(parentFrame);
   }

   public void changeParentFrame(ReferenceFrame parentFrame)
   {
      chestInteractableReferenceFrame.changeParentFrame(parentFrame);
   }

   public void setTransformToParent(Consumer<RigidBodyTransform> transformToParentConsumer)
   {
      chestInteractableReferenceFrame.update(transformToParentConsumer);
   }

   public RigidBodyTransform getTransformToParent()
   {
      return chestInteractableReferenceFrame.getTransformToParent();
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

   public ReferenceFrameLibrary getReferenceFrameLibrary()
   {
      return referenceFrameLibrary;
   }
}
