package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ChestOrientationActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalDouble trajectoryDuration;
   private final CRDTUnidirectionalBoolean holdPoseInWorldLater;
   private final CRDTUnidirectionalString parentFrameName;
   private final CRDTUnidirectionalRigidBodyTransform chestToParentTransform;

   // On disk fields
   private double onDiskTrajectoryDuration;
   private boolean onDiskHoldPoseInWorldLater;
   private String onDiskParentFrameName;
   private final RigidBodyTransform onDiskChestToParentTransform = new RigidBodyTransform();

   public ChestOrientationActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      trajectoryDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 4.0);
      holdPoseInWorldLater = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, false);
      parentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      chestToParentTransform = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater.getValue());
      jsonNode.put("parentFrame", parentFrameName.getValue());
      JSONTools.toJSON(jsonNode, chestToParentTransform.getValueReadOnly());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      holdPoseInWorldLater.setValue(jsonNode.get("holdPoseInWorldLater").asBoolean());
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, chestToParentTransform.getValue());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      onDiskHoldPoseInWorldLater = holdPoseInWorldLater.getValue();
      onDiskParentFrameName = parentFrameName.getValue();
      onDiskChestToParentTransform.set(chestToParentTransform.getValueReadOnly());
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      trajectoryDuration.setValue(onDiskTrajectoryDuration);
      holdPoseInWorldLater.setValue(onDiskHoldPoseInWorldLater);
      parentFrameName.setValue(onDiskParentFrameName);
      chestToParentTransform.getValue().set(onDiskChestToParentTransform);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
      unchanged &= holdPoseInWorldLater.getValue() == onDiskHoldPoseInWorldLater;
      unchanged &= parentFrameName.getValue().equals(onDiskParentFrameName);
      unchanged &= chestToParentTransform.getValue().equals(onDiskChestToParentTransform);

      return !unchanged;
   }

   public void toMessage(ChestOrientationActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      message.setHoldPoseInWorld(holdPoseInWorldLater.toMessage());
      message.setParentFrameName(parentFrameName.toMessage());
      chestToParentTransform.toMessage(message.getChestTransformToParent());
   }

   public void fromMessage(ChestOrientationActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      holdPoseInWorldLater.fromMessage(message.getHoldPoseInWorld());
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      chestToParentTransform.fromMessage(message.getChestTransformToParent());
   }

   public void setYaw(double yaw)
   {
      RotationMatrixBasics rotation = chestToParentTransform.getValue().getRotation();
      chestToParentTransform.getValue().getRotation().setYawPitchRoll(yaw, rotation.getPitch(), rotation.getRoll());
   }

   public void setPitch(double pitch)
   {
      RotationMatrixBasics rotation = chestToParentTransform.getValue().getRotation();
      chestToParentTransform.getValue().getRotation().setYawPitchRoll(rotation.getYaw(), pitch, rotation.getRoll());
   }

   public void setRoll(double roll)
   {
      RotationMatrixBasics rotation = chestToParentTransform.getValue().getRotation();
      chestToParentTransform.getValue().getRotation().setYawPitchRoll(rotation.getYaw(), rotation.getPitch(), roll);
   }

   public RotationMatrixBasics getRotation()
   {
      return chestToParentTransform.getValue().getRotation();
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration.getValue();
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration.setValue(trajectoryDuration);
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

   public CRDTUnidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTUnidirectionalRigidBodyTransform getChestToParentTransform()
   {
      return chestToParentTransform;
   }
}
