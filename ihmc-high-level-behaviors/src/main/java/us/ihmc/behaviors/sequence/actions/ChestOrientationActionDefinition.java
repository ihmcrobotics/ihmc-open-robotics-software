package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.BodyPartPoseActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.FrameBasedBehaviorActionDefinition;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.tools.io.JSONTools;

public class ChestOrientationActionDefinition extends FrameBasedBehaviorActionDefinition
{
   private String description = "Chest orientation";
   private double trajectoryDuration = 4.0;
   private boolean executeWitNextAction = false;
   private boolean holdPoseInWorldLater = false;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      JSONTools.toJSON(jsonNode, getTransformToParent());
      jsonNode.put("executeWithNextAction", executeWitNextAction);
      jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      getConditionalReferenceFrame().setParentFrameName(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, getTransformToParent());
      executeWitNextAction = jsonNode.get("executeWithNextAction").asBoolean();
      holdPoseInWorldLater = jsonNode.get("holdPoseInWorldLater").asBoolean();
   }

   public void toMessage(BodyPartPoseActionDefinitionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      MessageTools.toMessage(getTransformToParent(), message.getTransformToParent());
      message.setTrajectoryDuration(trajectoryDuration);
      message.setExecuteWithNextAction(executeWitNextAction);
      message.setHoldPoseInWorld(holdPoseInWorldLater);
   }

   public void fromMessage(BodyPartPoseActionDefinitionMessage message)
   {
      getConditionalReferenceFrame().setParentFrameName(message.getParentFrame().getString(0));
      MessageTools.toEuclid(message.getTransformToParent(), getTransformToParent());
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
