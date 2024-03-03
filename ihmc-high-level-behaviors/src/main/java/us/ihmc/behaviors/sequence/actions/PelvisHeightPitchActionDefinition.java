package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTUnidirectionalString;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class PelvisHeightPitchActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalDouble trajectoryDuration;
   private final CRDTUnidirectionalString parentFrameName;
   private final CRDTUnidirectionalRigidBodyTransform pelvisToParentTransform;

   // On disk fields
   private double onDiskTrajectoryDuration;
   private String onDiskParentFrameName;
   private final RigidBodyTransform onDiskPelvisToParentTransform = new RigidBodyTransform();

   public PelvisHeightPitchActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      trajectoryDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 4.0);
      parentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      pelvisToParentTransform = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      jsonNode.put("parentFrame", parentFrameName.getValue());
      JSONTools.toJSON(jsonNode, pelvisToParentTransform.getValueReadOnly());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, pelvisToParentTransform.getValue());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      onDiskParentFrameName = parentFrameName.getValue();
      onDiskPelvisToParentTransform.set(pelvisToParentTransform.getValueReadOnly());
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      trajectoryDuration.setValue(onDiskTrajectoryDuration);
      parentFrameName.setValue(onDiskParentFrameName);
      pelvisToParentTransform.getValue().set(onDiskPelvisToParentTransform);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
      unchanged &= parentFrameName.getValue().equals(onDiskParentFrameName);
      unchanged &= pelvisToParentTransform.getValueReadOnly().equals(onDiskPelvisToParentTransform);

      return !unchanged;
   }

   public void toMessage(PelvisHeightPitchActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      message.setParentFrameName(parentFrameName.toMessage());
      pelvisToParentTransform.toMessage(message.getPelvisTransformToParent());
   }

   public void fromMessage(PelvisHeightPitchActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      pelvisToParentTransform.fromMessage(message.getPelvisTransformToParent());
   }

   public void setHeight(double height)
   {
      pelvisToParentTransform.getValue().getTranslation().set(pelvisToParentTransform.getValue().getTranslationX(),
                                                              pelvisToParentTransform.getValue().getTranslationY(),
                                                              height);
   }

   public void setPitch(double pitch)
   {
      RotationMatrixBasics rotation = pelvisToParentTransform.getValue().getRotation();
      pelvisToParentTransform.getValue().getRotation().setYawPitchRoll(rotation.getYaw(), pitch, rotation.getRoll());
   }

   public double getHeight()
   {
      return pelvisToParentTransform.getValue().getTranslationZ();
   }

   public double getPitch()
   {
      return pelvisToParentTransform.getValue().getRotation().getPitch();
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration.getValue();
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration.setValue(trajectoryDuration);
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

   public CRDTUnidirectionalRigidBodyTransform getPelvisToParentTransform()
   {
      return pelvisToParentTransform;
   }
}
