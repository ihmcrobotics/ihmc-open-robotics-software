package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class HandPoseActionDefinition extends ActionNodeDefinition implements SidedObject
{
   public static final double DEFAULT_TRAJECTORY_DURATION = 4.0;
   public static final boolean DEFAULT_IS_JOINTSPACE_MODE = true;
   public static final boolean DEFAULT_HOLD_POSE =  false;
   public static final double DEFAULT_LINEAR_POSITION_WEIGHT = 50.0;
   public static final double DEFAULT_ANGULAR_POSITION_WEIGHT = 50.0;
   public static final double DEFAULT_JOINTSPACE_WEIGHT = -1.0;
   public static final double DEFAULT_POSITION_ERROR_TOLERANCE = 0.15;
   public static final double DEFAULT_ORIENTATION_ERROR_TOLERANCE = Math.toRadians(10.0);

   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalDouble trajectoryDuration;
   private final CRDTUnidirectionalBoolean holdPoseInWorldLater;
   private final CRDTUnidirectionalBoolean jointspaceOnly;
   private final CRDTUnidirectionalString palmParentFrameName;
   private final CRDTUnidirectionalRigidBodyTransform palmTransformToParent;
   private final CRDTUnidirectionalDouble linearPositionWeight;
   private final CRDTUnidirectionalDouble angularPositionWeight;
   private final CRDTUnidirectionalDouble jointspaceWeight;
   private final CRDTUnidirectionalDouble positionErrorTolerance;
   private final CRDTUnidirectionalDouble orientationErrorTolerance;

   // On disk fields
   private RobotSide onDiskSide;
   private double onDiskTrajectoryDuration;
   private boolean onDiskHoldPoseInWorldLater;
   private boolean onDiskJointspaceOnly;
   private String onDiskPalmParentFrameName;
   private final RigidBodyTransform onDiskPalmTransformToParent = new RigidBodyTransform();
   private double onDiskLinearPositionWeight;
   private double onDiskAngularPositionWeight;
   private double onDiskJointspaceWeight;
   private double onDiskPositionErrorTolerance;
   private double onDiskOrientationErrorTolerance;

   public HandPoseActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      trajectoryDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_TRAJECTORY_DURATION);
      holdPoseInWorldLater = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_HOLD_POSE);
      jointspaceOnly = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_IS_JOINTSPACE_MODE);
      palmParentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      palmTransformToParent = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
      linearPositionWeight = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_LINEAR_POSITION_WEIGHT);
      angularPositionWeight = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_ANGULAR_POSITION_WEIGHT);
      jointspaceWeight = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_JOINTSPACE_WEIGHT);
      positionErrorTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_POSITION_ERROR_TOLERANCE);
      orientationErrorTolerance = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_ORIENTATION_ERROR_TOLERANCE);

   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("parentFrame", palmParentFrameName.getValue());
      JSONTools.toJSON(jsonNode, palmTransformToParent.getValueReadOnly());
      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater.getValue());
      jsonNode.put("jointspaceOnly", jointspaceOnly.getValue());
      jsonNode.put("linearPositionWeight", linearPositionWeight.getValue());
      jsonNode.put("angularPositionWeight", angularPositionWeight.getValue());
      jsonNode.put("jointspaceWeight", jointspaceWeight.getValue());
      jsonNode.put("positionErrorTolerance", Double.parseDouble("%.3f".formatted(positionErrorTolerance.getValue())));
      jsonNode.put("orientationErrorToleranceDegrees", Double.parseDouble("%.3f".formatted(Math.toDegrees(orientationErrorTolerance.getValue()))));
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      palmParentFrameName.setValue(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, palmTransformToParent.getValue());
      holdPoseInWorldLater.setValue(jsonNode.get("holdPoseInWorldLater").asBoolean());
      jointspaceOnly.setValue(jsonNode.get("jointspaceOnly").asBoolean());
      linearPositionWeight.setValue(jsonNode.get("linearPositionWeight").asDouble());
      angularPositionWeight.setValue(jsonNode.get("angularPositionWeight").asDouble());
      jointspaceWeight.setValue(jsonNode.get("jointspaceWeight").asDouble());
      positionErrorTolerance.setValue(jsonNode.get("positionErrorTolerance").asDouble());
      orientationErrorTolerance.setValue(Math.toRadians(jsonNode.get("orientationErrorToleranceDegrees").asDouble()));
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      onDiskHoldPoseInWorldLater = holdPoseInWorldLater.getValue();
      onDiskJointspaceOnly = jointspaceOnly.getValue();
      onDiskPalmParentFrameName = palmParentFrameName.getValue();
      onDiskPalmTransformToParent.set(palmTransformToParent.getValueReadOnly());
      onDiskLinearPositionWeight = linearPositionWeight.getValue();
      onDiskAngularPositionWeight = angularPositionWeight.getValue();
      onDiskJointspaceWeight = jointspaceWeight.getValue();
      onDiskPositionErrorTolerance = positionErrorTolerance.getValue();
      onDiskOrientationErrorTolerance = orientationErrorTolerance.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      side.setValue(onDiskSide);
      trajectoryDuration.setValue(onDiskTrajectoryDuration);
      holdPoseInWorldLater.setValue(onDiskHoldPoseInWorldLater);
      jointspaceOnly.setValue(onDiskJointspaceOnly);
      palmParentFrameName.setValue(onDiskPalmParentFrameName);
      palmTransformToParent.getValue().set(onDiskPalmTransformToParent);
      linearPositionWeight.setValue(onDiskLinearPositionWeight);
      angularPositionWeight.setValue(onDiskAngularPositionWeight);
      jointspaceWeight.setValue(onDiskJointspaceWeight);
      positionErrorTolerance.setValue(onDiskPositionErrorTolerance);
      orientationErrorTolerance.setValue(onDiskOrientationErrorTolerance);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
      unchanged &= holdPoseInWorldLater.getValue() == onDiskHoldPoseInWorldLater;
      unchanged &= jointspaceOnly.getValue() == onDiskJointspaceOnly;
      unchanged &= palmParentFrameName.getValue().equals(onDiskPalmParentFrameName);
      unchanged &= palmTransformToParent.getValueReadOnly().equals(onDiskPalmTransformToParent);
      unchanged &= linearPositionWeight.getValue() == onDiskLinearPositionWeight;
      unchanged &= angularPositionWeight.getValue() == onDiskAngularPositionWeight;
      unchanged &= jointspaceWeight.getValue() == onDiskJointspaceWeight;
      unchanged &= positionErrorTolerance.getValue() == onDiskPositionErrorTolerance;
      unchanged &= orientationErrorTolerance.getValue() == onDiskOrientationErrorTolerance;

      return !unchanged;
   }

   public void toMessage(HandPoseActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setParentFrameName(palmParentFrameName.toMessage());
      palmTransformToParent.toMessage(message.getTransformToParent());
      message.setRobotSide(side.toMessage().toByte());
      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      message.setHoldPoseInWorld(holdPoseInWorldLater.toMessage());
      message.setJointSpaceControl(jointspaceOnly.toMessage());
      message.setLinearPositionWeight(linearPositionWeight.toMessage());
      message.setAngularPositionWeight(angularPositionWeight.toMessage());
      message.setJointspaceWeight(jointspaceWeight.toMessage());
      message.setPositionErrorTolerance(positionErrorTolerance.toMessage());
      message.setOrientationErrorTolerance(orientationErrorTolerance.toMessage());
   }

   public void fromMessage(HandPoseActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      palmParentFrameName.fromMessage(message.getParentFrameNameAsString());
      palmTransformToParent.fromMessage(message.getTransformToParent());
      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      holdPoseInWorldLater.fromMessage(message.getHoldPoseInWorld());
      jointspaceOnly.fromMessage(message.getJointSpaceControl());
      linearPositionWeight.fromMessage(message.getLinearPositionWeight());
      angularPositionWeight.fromMessage(message.getAngularPositionWeight());
      jointspaceWeight.fromMessage(message.getJointspaceWeight());
      positionErrorTolerance.fromMessage(message.getPositionErrorTolerance());
      orientationErrorTolerance.fromMessage(message.getOrientationErrorTolerance());
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

   public boolean getHoldPoseInWorldLater()
   {
      return holdPoseInWorldLater.getValue();
   }

   public void setHoldPoseInWorldLater(boolean holdPoseInWorldLater)
   {
      this.holdPoseInWorldLater.setValue(holdPoseInWorldLater);
   }

   public boolean getJointspaceOnly()
   {
      return jointspaceOnly.getValue();
   }

   public void setJointspaceOnly(boolean jointspaceOnly)
   {
      this.jointspaceOnly.setValue(jointspaceOnly);
   }

   public String getPalmParentFrameName()
   {
      return palmParentFrameName.getValue();
   }

   public void setPalmParentFrameName(String palmParentFrameName)
   {
      this.palmParentFrameName.setValue(palmParentFrameName);
   }

   public CRDTUnidirectionalString getCRDTPalmParentFrameName()
   {
      return palmParentFrameName;
   }

   public CRDTUnidirectionalRigidBodyTransform getPalmTransformToParent()
   {
      return palmTransformToParent;
   }

   public double getLinearPositionWeight()
   {
      return linearPositionWeight.getValue();
   }

   public void setLinearPositionWeight(double linearPositionWeight)
   {
      this.linearPositionWeight.setValue(linearPositionWeight);
   }

   public double getAngularPositionWeight()
   {
      return angularPositionWeight.getValue();
   }

   public void setAngularPositionWeight(double angularPositionWeight)
   {
      this.angularPositionWeight.setValue(angularPositionWeight);
   }

   public double getJointspaceWeight()
   {
      return jointspaceWeight.getValue();
   }

   public void setJointspaceWeight(double jointspaceWeight)
   {
      this.jointspaceWeight.setValue(jointspaceWeight);
   }

   public double getPositionErrorTolerance()
   {
      return positionErrorTolerance.getValue();
   }

   public void setPositionErrorTolerance(double positionErrorTolerance)
   {
      this.positionErrorTolerance.setValue(positionErrorTolerance);
   }

   public double getOrientationErrorTolerance()
   {
      return orientationErrorTolerance.getValue();
   }

   public void setOrientationErrorTolerance(double orientationErrorTolerance)
   {
      this.orientationErrorTolerance.setValue(orientationErrorTolerance);
   }
}
