package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.BodyPartPoseActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;

public class PelvisHeightPitchActionDefinition implements BehaviorActionDefinition<BodyPartPoseActionDefinitionMessage>
{
   private String description = "Pelvis height and pitch";
   private double trajectoryDuration = 4.0;
   private boolean executeWitNextAction = false;
   private String parentFrameName;
   private final RigidBodyTransform pelvisToParentTransform = new RigidBodyTransform();

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("executeWithNextAction", executeWitNextAction);
      jsonNode.put("parentFrame", parentFrameName);
      JSONTools.toJSON(jsonNode, pelvisToParentTransform);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      executeWitNextAction = jsonNode.get("executeWithNextAction").asBoolean();
      parentFrameName = jsonNode.get("parentFrame").textValue();
      JSONTools.toEuclid(jsonNode, pelvisToParentTransform);
   }

   @Override
   public void toMessage(BodyPartPoseActionDefinitionMessage message)
   {
      message.setTrajectoryDuration(trajectoryDuration);
      message.setExecuteWithNextAction(executeWitNextAction);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(parentFrameName);
      MessageTools.toMessage(pelvisToParentTransform, message.getTransformToParent());
   }

   @Override
   public void fromMessage(BodyPartPoseActionDefinitionMessage message)
   {
      trajectoryDuration = message.getTrajectoryDuration();
      executeWitNextAction = message.getExecuteWithNextAction();
      parentFrameName = message.getParentFrame().getString(0);
      MessageTools.toEuclid(message.getTransformToParent(), pelvisToParentTransform);
   }

   public void setHeight(double height)
   {
      pelvisToParentTransform.getTranslation().set(pelvisToParentTransform.getTranslationX(), pelvisToParentTransform.getTranslationY(), height);
   }

   public void setPitch(double pitch)
   {
      RotationMatrixBasics rotation = pelvisToParentTransform.getRotation();
      pelvisToParentTransform.getRotation().setYawPitchRoll(rotation.getYaw(), pitch, rotation.getRoll());
   }

   public double getHeight()
   {
      return pelvisToParentTransform.getTranslationZ();
   }

   public double getPitch()
   {
      return pelvisToParentTransform.getRotation().getPitch();
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
   public boolean getExecuteWithNextAction()
   {
      return executeWitNextAction;
   }

   public void setExecuteWithNextAction(boolean executeWitNextAction)
   {
      this.executeWitNextAction = executeWitNextAction;
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

   public String getParentFrameName()
   {
      return parentFrameName;
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName = parentFrameName;
   }

   public RigidBodyTransform getPelvisToParentTransform()
   {
      return pelvisToParentTransform;
   }
}
