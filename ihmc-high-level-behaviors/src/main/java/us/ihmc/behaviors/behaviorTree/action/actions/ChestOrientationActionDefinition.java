package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.BooleanNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.crdt.*;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;

public class ChestOrientationActionDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalString parentFrameName;
   private final CRDTBidirectionalDouble trajectoryDuration;
   private final CRDTBidirectionalBoolean jointspaceOnly;
   private final CRDTBidirectionalDoubleArray jointAngles;
   private final CRDTBidirectionalRigidBodyTransform chestToParentTransform;
   private final CRDTBidirectionalBoolean holdPoseInWorldLater;

   // On disk fields
   private String onDiskParentFrameName;
   private double onDiskTrajectoryDuration;
   private boolean onDiskJointspaceOnly;
   private final double[] onDiskJointAngles = new double[3];
   private final RigidBodyTransform onDiskChestToParentTransform = new RigidBodyTransform();
   private boolean onDiskHoldPoseInWorldLater;

   public ChestOrientationActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      parentFrameName = new CRDTBidirectionalString(this, "Pelvis");
      trajectoryDuration = new CRDTBidirectionalDouble(this, 4.0);
      jointspaceOnly = new CRDTBidirectionalBoolean(this, true);
      jointAngles = new CRDTBidirectionalDoubleArray(this, 3);
      chestToParentTransform = new CRDTBidirectionalRigidBodyTransform(this);
      holdPoseInWorldLater = new CRDTBidirectionalBoolean(this, false);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("parentFrame", parentFrameName.getValue());
      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      jsonNode.put("jointspaceOnly", jointspaceOnly.getValue());
      if (jointspaceOnly.getValue())
         for (int i = 0; i < jointAngles.getLength(); i++)
            jsonNode.put("j" + i, (float) MathTools.roundToPrecision(Math.toDegrees(jointAngles.getValueReadOnly(i)), 0.02));
      else
      {
         JSONTools.toJSON(jsonNode, chestToParentTransform.getValueReadOnly());
         jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater.getValue());
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      if (jsonNode.get("jointspaceOnly") instanceof BooleanNode)
      {
         jointspaceOnly.setValue(jsonNode.get("jointspaceOnly").asBoolean());
         for (int i = 0; i < jointAngles.getLength(); i++)
            jointAngles.setValue(i, Math.toRadians(jsonNode.get("j" + i).asDouble()));
      }
      else
      {
         JSONTools.toEuclid(jsonNode, chestToParentTransform.getValueAndModify());
         holdPoseInWorldLater.setValue(jsonNode.get("holdPoseInWorldLater").asBoolean());
      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskParentFrameName = parentFrameName.getValue();
      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      onDiskJointspaceOnly = jointspaceOnly.getValue();
      for (int i = 0; i < jointAngles.getLength(); i++)
         onDiskJointAngles[i] = jointAngles.getValueReadOnly(i);
      onDiskChestToParentTransform.set(chestToParentTransform.getValueReadOnly());
      onDiskHoldPoseInWorldLater = holdPoseInWorldLater.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         parentFrameName.setValue(onDiskParentFrameName);
         trajectoryDuration.setValue(onDiskTrajectoryDuration);
         jointspaceOnly.setValue(onDiskJointspaceOnly);
         for (int i = 0; i < jointAngles.getLength(); i++)
            jointAngles.setValue(i, onDiskJointAngles[i]);
         chestToParentTransform.getValueAndModify().set(onDiskChestToParentTransform);
         holdPoseInWorldLater.setValue(onDiskHoldPoseInWorldLater);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= parentFrameName.getValue().equals(onDiskParentFrameName);
      unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
      unchanged &= jointspaceOnly.getValue() == onDiskJointspaceOnly;
      for (int i = 0; i < jointAngles.getLength(); i++)
         unchanged &= jointAngles.getValueReadOnly(i) == onDiskJointAngles[i];
      unchanged &= chestToParentTransform.getValueReadOnly().equals(onDiskChestToParentTransform);
      unchanged &= holdPoseInWorldLater.getValue() == onDiskHoldPoseInWorldLater;

      return !unchanged;
   }

   public void toMessage(ChestOrientationActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setParentFrameName(parentFrameName.toMessage());
      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      message.setJointspaceOnly(jointspaceOnly.toMessage());
      jointAngles.toMessage(message.getJointAngles());
      chestToParentTransform.toMessage(message.getChestTransformToParent());
      message.setHoldPoseInWorld(holdPoseInWorldLater.toMessage());
   }

   public void fromMessage(ChestOrientationActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      jointspaceOnly.fromMessage(message.getJointspaceOnly());
      jointAngles.fromMessage(message.getJointAngles());
      chestToParentTransform.fromMessage(message.getChestTransformToParent());
      holdPoseInWorldLater.fromMessage(message.getHoldPoseInWorld());
   }

   public void setYaw(double yaw)
   {
      RotationMatrixBasics rotation = chestToParentTransform.getValueAndModify().getRotation();
      chestToParentTransform.getValueAndModify().getRotation().setYawPitchRoll(yaw, rotation.getPitch(), rotation.getRoll());
   }

   public void setPitch(double pitch)
   {
      RotationMatrixBasics rotation = chestToParentTransform.getValueAndModify().getRotation();
      chestToParentTransform.getValueAndModify().getRotation().setYawPitchRoll(rotation.getYaw(), pitch, rotation.getRoll());
   }

   public void setRoll(double roll)
   {
      RotationMatrixBasics rotation = chestToParentTransform.getValueAndModify().getRotation();
      chestToParentTransform.getValueAndModify().getRotation().setYawPitchRoll(rotation.getYaw(), rotation.getPitch(), roll);
   }

   public RotationMatrixBasics getRotation()
   {
      return chestToParentTransform.getValueAndModify().getRotation();
   }

   public RotationMatrixReadOnly getRotationReadOnly()
   {
      return (RotationMatrixReadOnly) chestToParentTransform.getValueReadOnly().getRotation();
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration.getValue();
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration.setValue(trajectoryDuration);
   }

   public boolean getJointspaceOnly()
   {
      return jointspaceOnly.getValue();
   }

   public void setJointspaceOnly(boolean jointspaceOnly)
   {
      this.jointspaceOnly.setValue(jointspaceOnly);
   }

   public CRDTBidirectionalDoubleArray getJointAngles()
   {
      return jointAngles;
   }

   public boolean getHoldPoseInWorldLater()
   {
      return holdPoseInWorldLater.getValue();
   }

   public void setHoldPoseInWorldLater(boolean holdPoseInWorldLater)
   {
      this.holdPoseInWorldLater.setValue(holdPoseInWorldLater);
   }

   public String getParentFrameName()
   {
      return parentFrameName.getValue();
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName.setValue(parentFrameName);
   }

   public CRDTBidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTBidirectionalRigidBodyTransform getChestToParentTransform()
   {
      return chestToParentTransform;
   }
}
