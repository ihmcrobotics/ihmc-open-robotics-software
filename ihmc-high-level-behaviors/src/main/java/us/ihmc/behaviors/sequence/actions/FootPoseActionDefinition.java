package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootPoseActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootPoseActionDefinition extends ActionNodeDefinition implements SidedObject
{
   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalDouble trajectoryDuration;
   private final CRDTUnidirectionalString parentFrameName;
   private final CRDTUnidirectionalRigidBodyTransform footToParentTransform;

   // On disk fields
   private RobotSide onDiskSide;
   private double onDiskTrajectoryDuration;
   private String onDiskParentFrameName;
   private final RigidBodyTransform onDiskFootToParentTransform = new RigidBodyTransform();

   public FootPoseActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      trajectoryDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 4.0);
      parentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      footToParentTransform = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      jsonNode.put("parentFrame", parentFrameName.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, footToParentTransform.getValue());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      onDiskParentFrameName = parentFrameName.getValue();
      onDiskFootToParentTransform.set(footToParentTransform.getValueReadOnly());
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      side.setValue(onDiskSide);
      trajectoryDuration.setValue(onDiskTrajectoryDuration);
      parentFrameName.setValue(onDiskParentFrameName);
      footToParentTransform.getValue().set(onDiskFootToParentTransform);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
      unchanged &= parentFrameName.getValue().equals(onDiskParentFrameName);
      unchanged &= footToParentTransform.getValueReadOnly().equals(onDiskFootToParentTransform);

      return !unchanged;
   }

   public void toMessage(FootPoseActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setParentFrameName(parentFrameName.toMessage());
      footToParentTransform.toMessage(message.getTransformToParent());
      message.setTrajectoryDuration(trajectoryDuration.toMessage());
   }

   public void fromMessage(FootPoseActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      parentFrameName.fromMessage(message.getParentFrameNameAsString());
      footToParentTransform.fromMessage(message.getTransformToParent());
      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
   }

   public RotationMatrixBasics getRotation()
   {
      return footToParentTransform.getValue().getRotation();
   }

   @Override
   public RobotSide getSide()
   {
      return side.getValue();
   }

   public void setSide(RobotSide side)
   {
      this.side.setValue(side);
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

   public CRDTUnidirectionalRigidBodyTransform getFootToParentTransform()
   {
      return footToParentTransform;
   }
}
