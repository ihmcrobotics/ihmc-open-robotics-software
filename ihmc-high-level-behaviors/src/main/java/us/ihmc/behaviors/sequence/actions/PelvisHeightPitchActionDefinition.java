package us.ihmc.behaviors.sequence.actions;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;

public class PelvisHeightPitchActionDefinition extends BehaviorActionDefinition
{
   private double trajectoryDuration = 4.0;
   private String parentFrameName;
   private final RigidBodyTransform pelvisToParentTransform = new RigidBodyTransform();

   public PelvisHeightPitchActionDefinition()
   {
      super("Pelvis height and pitch");
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("parentFrame", parentFrameName);
      JSONTools.toJSON(jsonNode, pelvisToParentTransform);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      parentFrameName = jsonNode.get("parentFrame").textValue();
      JSONTools.toEuclid(jsonNode, pelvisToParentTransform);
   }

   public void toMessage(BodyPartPoseActionDefinitionMessage message)
   {
      super.toMessage(message.getActionDefinition());

      message.setTrajectoryDuration(trajectoryDuration);
      message.setExecuteWithNextAction(getExecuteWithNextAction());
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(parentFrameName);
      MessageTools.toMessage(pelvisToParentTransform, message.getTransformToParent());
   }

   public void fromMessage(BodyPartPoseActionDefinitionMessage message)
   {
      super.fromMessage(message.getActionDefinition());

      trajectoryDuration = message.getTrajectoryDuration();
      setExecuteWithNextAction(message.getExecuteWithNextAction());
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
