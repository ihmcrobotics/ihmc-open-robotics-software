package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.BodyPartPoseActionDescriptionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.FrameBasedBehaviorActionData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.tools.io.JSONTools;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class PelvisHeightPitchActionDescription extends FrameBasedBehaviorActionData
{
   private String description = "Pelvis height and pitch";
   private double trajectoryDuration = 4.0;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ModifiableReferenceFrame pelvisInteractableReferenceFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private boolean executeWitNextAction = false;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", pelvisInteractableReferenceFrame.getReferenceFrame().getParent().getName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      JSONTools.toJSON(jsonNode, pelvisInteractableReferenceFrame.getTransformToParent());
      jsonNode.put("executeWithNextAction", executeWitNextAction);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      pelvisInteractableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(jsonNode.get("parentFrame").asText()));
      pelvisInteractableReferenceFrame.update(transformToParent -> JSONTools.toEuclid(jsonNode, transformToParent));
      executeWitNextAction = jsonNode.get("executeWithNextAction").asBoolean();
   }

   public void toMessage(BodyPartPoseActionDescriptionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getParentFrame().getName());
      MessageTools.toMessage(pelvisInteractableReferenceFrame.getTransformToParent(), message.getTransformToParent());
      message.setTrajectoryDuration(trajectoryDuration);
      message.setExecuteWithNextAction(executeWitNextAction);
   }

   public void fromMessage(BodyPartPoseActionDescriptionMessage message)
   {
      pelvisInteractableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(message.getParentFrame().getString(0)));
      pelvisInteractableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
      trajectoryDuration = message.getTrajectoryDuration();
      executeWitNextAction = message.getExecuteWithNextAction();
   }

   public void setHeight(double height)
   {
      getTransformToParent().getTranslation().set(getTransformToParent().getTranslationX(), getTransformToParent().getTranslationY(), height);
   }

   public void setPitch(double pitch)
   {
      RotationMatrixBasics rotation = getTransformToParent().getRotation();
      getTransformToParent().getRotation().setYawPitchRoll(rotation.getYaw(), pitch, rotation.getRoll());
   }

   public double getHeight()
   {
      return getTransformToParent().getTranslationZ();
   }

   public double getPitch()
   {
      return getTransformToParent().getRotation().getPitch();
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
   public boolean getExecuteWithNextAction()
   {
      return executeWitNextAction;
   }

   public void setExecuteWithNextAction(boolean executeWitNextAction)
   {
      this.executeWitNextAction = executeWitNextAction;
   }

   public ReferenceFrame getParentFrame()
   {
      jsonNode.put("description", description);
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("parentFrame", getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      JSONTools.toJSON(jsonNode, getTransformToParent());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      getConditionalReferenceFrame().setParentFrameName(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, getTransformToParent());
   }

   public void toMessage(BodyPartPoseActionDescriptionMessage message)
   {
      message.setTrajectoryDuration(trajectoryDuration);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      MessageTools.toMessage(getTransformToParent(), message.getTransformToParent());
   }

   public void fromMessage(BodyPartPoseActionDescriptionMessage message)
   {
      trajectoryDuration = message.getTrajectoryDuration();
      getConditionalReferenceFrame().setParentFrameName(message.getParentFrame().getString(0));
      MessageTools.toEuclid(message.getTransformToParent(), getTransformToParent());
   }
}
