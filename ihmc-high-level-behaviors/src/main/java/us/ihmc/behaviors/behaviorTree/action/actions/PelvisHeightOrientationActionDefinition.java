package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.PelvisHeightOrientationActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.JSONTools;

public class PelvisHeightOrientationActionDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalDouble trajectoryDuration;
   private final CRDTBidirectionalString parentFrameName;
   private final CRDTBidirectionalRigidBodyTransform pelvisToParentTransform;

   // On disk fields
   private double onDiskTrajectoryDuration;
   private String onDiskParentFrameName;
   private final RigidBodyTransform onDiskPelvisToParentTransform = new RigidBodyTransform();

   public PelvisHeightOrientationActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      trajectoryDuration = new CRDTBidirectionalDouble(this, 4.0);
      parentFrameName = new CRDTBidirectionalString(this, "Walking");
      pelvisToParentTransform = new CRDTBidirectionalRigidBodyTransform(this);
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
      JSONTools.toEuclid(jsonNode, pelvisToParentTransform.getValueAndModify());
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

      if (isUndoAvailable())
      {
         trajectoryDuration.setValue(onDiskTrajectoryDuration);
         parentFrameName.setValue(onDiskParentFrameName);
         pelvisToParentTransform.getValueAndModify().set(onDiskPelvisToParentTransform);
      }
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

   public void toMessage(PelvisHeightOrientationActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      message.setParentFrameName(parentFrameName.toMessage());
      pelvisToParentTransform.toMessage(message.getPelvisTransformToParent());
   }

   public void fromMessage(PelvisHeightOrientationActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      pelvisToParentTransform.fromMessage(message.getPelvisTransformToParent());
   }

   public void setHeight(double height)
   {
      pelvisToParentTransform.getValueAndModify().getTranslation().set(pelvisToParentTransform.getValueAndModify().getTranslationX(),
                                                                       pelvisToParentTransform.getValueAndModify().getTranslationY(),
                                                                       height);
   }

   public void setYaw(double yaw)
   {
      RotationMatrixBasics rotation = pelvisToParentTransform.getValueAndModify().getRotation();
      pelvisToParentTransform.getValueAndModify().getRotation().setYawPitchRoll(yaw, rotation.getPitch(), rotation.getRoll());
   }

   public void setPitch(double pitch)
   {
      RotationMatrixBasics rotation = pelvisToParentTransform.getValueAndModify().getRotation();
      pelvisToParentTransform.getValueAndModify().getRotation().setYawPitchRoll(rotation.getYaw(), pitch, rotation.getRoll());
   }

   public void setRoll(double roll)
   {
      RotationMatrixBasics rotation = pelvisToParentTransform.getValueAndModify().getRotation();
      pelvisToParentTransform.getValueAndModify().getRotation().setYawPitchRoll(rotation.getYaw(), rotation.getPitch(), roll);
   }

   public Orientation3DReadOnly getRotation()
   {
      return pelvisToParentTransform.getValueReadOnly().getRotation();
   }

   public double getHeight()
   {
      return pelvisToParentTransform.getValueReadOnly().getTranslationZ();
   }

   public double getPitch()
   {
      return pelvisToParentTransform.getValueReadOnly().getRotation().getPitch();
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

   public CRDTBidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTBidirectionalRigidBodyTransform getPelvisToParentTransform()
   {
      return pelvisToParentTransform;
   }
}
