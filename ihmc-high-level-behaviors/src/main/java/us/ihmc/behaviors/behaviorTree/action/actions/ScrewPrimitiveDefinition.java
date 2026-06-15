package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.ArmActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;

public class ScrewPrimitiveDefinition
{
   public static final double DEFAULT_TRANSLATION = 0.1;
   public static final double DEFAULT_ROTATION = 0.0;
   public static final double DEFAULT_MAX_LINEAR_VELOCITY = 0.1;
   public static final double DEFAULT_MAX_ANGULAR_VELOCITY = 0.6;

   private final CRDTBidirectionalRigidBodyTransform screwAxisPoseInObjectFrame;
   private final CRDTBidirectionalDouble translation;
   private final CRDTBidirectionalDouble rotation;
   private final CRDTBidirectionalDouble maxLinearVelocity;
   private final CRDTBidirectionalDouble maxAngularVelocity;

   private final RigidBodyTransform onDiskScrewAxisPoseInObjectFrame = new RigidBodyTransform();
   private double onDiskTranslation;
   private double onDiskRotation;
   private double onDiskMaxLinearVelocity;
   private double onDiskMaxAngularVelocity;
   private boolean onDiskFieldsSet = false;

   public ScrewPrimitiveDefinition(LatestTimestampModifiable latestTimestampModifiable)
   {
      screwAxisPoseInObjectFrame = new CRDTBidirectionalRigidBodyTransform(latestTimestampModifiable);
      translation = new CRDTBidirectionalDouble(latestTimestampModifiable, DEFAULT_TRANSLATION);
      rotation = new CRDTBidirectionalDouble(latestTimestampModifiable, DEFAULT_ROTATION);
      maxLinearVelocity = new CRDTBidirectionalDouble(latestTimestampModifiable, DEFAULT_MAX_LINEAR_VELOCITY);
      maxAngularVelocity = new CRDTBidirectionalDouble(latestTimestampModifiable, DEFAULT_MAX_ANGULAR_VELOCITY);
   }

   public void saveToFile(ObjectNode jsonNode, double orientationErrorToleranceDegrees)
   {
      JSONTools.toJSON(jsonNode.putObject("screwAxisPose"), screwAxisPoseInObjectFrame.getValueReadOnly());
      jsonNode.put("translation", translation.getValue());
      jsonNode.put("rotation", rotation.getValue());
      jsonNode.put("maxLinearVelocity", maxLinearVelocity.getValue());
      jsonNode.put("maxAngularVelocity", maxAngularVelocity.getValue());
      jsonNode.put("orientationErrorToleranceDegrees", orientationErrorToleranceDegrees);
   }

   public void loadFromFile(JsonNode jsonNode)
   {
      if (jsonNode.get("screwAxisPose") instanceof ObjectNode objectNode)
         JSONTools.toEuclid(objectNode, screwAxisPoseInObjectFrame.getValueAndModify());
      translation.setValue(jsonNode.get("translation").asDouble());
      rotation.setValue(jsonNode.get("rotation").asDouble());
      maxLinearVelocity.setValue(jsonNode.get("maxLinearVelocity").asDouble());
      maxAngularVelocity.setValue(jsonNode.get("maxAngularVelocity").asDouble());
   }

   public void setOnDiskFields()
   {
      onDiskScrewAxisPoseInObjectFrame.set(screwAxisPoseInObjectFrame.getValueReadOnly());
      onDiskTranslation = translation.getValue();
      onDiskRotation = rotation.getValue();
      onDiskMaxLinearVelocity = maxLinearVelocity.getValue();
      onDiskMaxAngularVelocity = maxAngularVelocity.getValue();
      onDiskFieldsSet = true;
   }

   public void undoAllNontopologicalChanges()
   {
      if (onDiskFieldsSet)
      {
         screwAxisPoseInObjectFrame.getValueAndModify().set(onDiskScrewAxisPoseInObjectFrame);
         translation.setValue(onDiskTranslation);
         rotation.setValue(onDiskRotation);
         maxLinearVelocity.setValue(onDiskMaxLinearVelocity);
         maxAngularVelocity.setValue(onDiskMaxAngularVelocity);
      }
   }

   public boolean hasChanges()
   {
      if (!onDiskFieldsSet)
         return false;

      boolean unchanged = true;
      unchanged &= screwAxisPoseInObjectFrame.getValueReadOnly().equals(onDiskScrewAxisPoseInObjectFrame);
      unchanged &= translation.getValue() == onDiskTranslation;
      unchanged &= rotation.getValue() == onDiskRotation;
      unchanged &= maxLinearVelocity.getValue() == onDiskMaxLinearVelocity;
      unchanged &= maxAngularVelocity.getValue() == onDiskMaxAngularVelocity;
      return !unchanged;
   }

   public void toMessage(ArmActionDefinitionMessage message)
   {
      screwAxisPoseInObjectFrame.toMessage(message.getScrewAxisPose());
      message.setTranslation(translation.toMessage());
      message.setRotation(rotation.toMessage());
      message.setMaxLinearVelocity(maxLinearVelocity.toMessage());
      message.setMaxAngularVelocity(maxAngularVelocity.toMessage());
   }

   public void fromMessage(ArmActionDefinitionMessage message)
   {
      screwAxisPoseInObjectFrame.fromMessage(message.getScrewAxisPose());
      translation.fromMessage(message.getTranslation());
      rotation.fromMessage(message.getRotation());
      maxLinearVelocity.fromMessage(message.getMaxLinearVelocity());
      maxAngularVelocity.fromMessage(message.getMaxAngularVelocity());
   }

   public CRDTBidirectionalRigidBodyTransform getScrewAxisPoseInObjectFrame()
   {
      return screwAxisPoseInObjectFrame;
   }

   public double getTranslation()
   {
      return translation.getValue();
   }

   public void setTranslation(double translation)
   {
      this.translation.setValue(translation);
   }

   public double getRotation()
   {
      return rotation.getValue();
   }

   public void setRotation(double rotation)
   {
      this.rotation.setValue(rotation);
   }

   public double getMaxLinearVelocity()
   {
      return maxLinearVelocity.getValue();
   }

   public void setMaxLinearVelocity(double maxLinearVelocity)
   {
      this.maxLinearVelocity.setValue(maxLinearVelocity);
   }

   public double getMaxAngularVelocity()
   {
      return maxAngularVelocity.getValue();
   }

   public void setMaxAngularVelocity(double maxAngularVelocity)
   {
      this.maxAngularVelocity.setValue(maxAngularVelocity);
   }
}
