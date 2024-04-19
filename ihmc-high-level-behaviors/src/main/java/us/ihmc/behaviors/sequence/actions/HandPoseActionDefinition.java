package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

public class HandPoseActionDefinition extends ActionNodeDefinition implements SidedObject
{
   public static final double DEFAULT_TRAJECTORY_DURATION = 4.0;
   public static final boolean DEFAULT_IS_JOINTSPACE_MODE = true;
   public static final boolean DEFAULT_USE_PREDEFINED_JOINT_ANGLES = false;
   public static final int MAX_NUMBER_OF_JOINTS = 7;
   public static final String CUSTOM_ANGLES_NAME = "CUSTOM_ANGLES";
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
   private final CRDTUnidirectionalBoolean usePredefinedJointAngles;
   /** Preset is null when using explicitly specified custom joint angles */
   private final CRDTUnidirectionalEnumField<PresetArmConfiguration> preset;
   private final CRDTUnidirectionalDoubleArray jointAngles;
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
   private boolean onDiskUsePredefinedJointAngles;
   private PresetArmConfiguration onDiskPreset;
   private final double[] onDiskJointAngles = new double[MAX_NUMBER_OF_JOINTS];
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
      usePredefinedJointAngles = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, DEFAULT_USE_PREDEFINED_JOINT_ANGLES);
      preset = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, PresetArmConfiguration.HOME);
      jointAngles = new CRDTUnidirectionalDoubleArray(ROS2ActorDesignation.OPERATOR, crdtInfo, MAX_NUMBER_OF_JOINTS);
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

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      jsonNode.put("usePredefinedJointAngles", usePredefinedJointAngles.getValue());

      if (usePredefinedJointAngles.getValue())
      {
         jsonNode.put("preset", preset.getValue() == null ? CUSTOM_ANGLES_NAME : preset.getValue().name());
         if (preset.getValue() == null)
         {
            for (int i = 0; i < MAX_NUMBER_OF_JOINTS; i++)
            {
               jsonNode.put("j" + i, jointAngles.getValueReadOnly(i));
            }
         }
      }
      else
      {
         jsonNode.put("parentFrame", palmParentFrameName.getValue());
         JSONTools.toJSON(jsonNode, palmTransformToParent.getValueReadOnly());
         jsonNode.put("jointspaceOnly", jointspaceOnly.getValue());
         jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater.getValue());
         jsonNode.put("linearPositionWeight", linearPositionWeight.getValue());
         jsonNode.put("angularPositionWeight", angularPositionWeight.getValue());
         jsonNode.put("positionErrorTolerance", Double.parseDouble("%.3f".formatted(positionErrorTolerance.getValue())));
         jsonNode.put("orientationErrorToleranceDegrees", Double.parseDouble("%.3f".formatted(Math.toDegrees(orientationErrorTolerance.getValue()))));
      }

      jsonNode.put("jointspaceWeight", jointspaceWeight.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      usePredefinedJointAngles.setValue(jsonNode.get("usePredefinedJointAngles").asBoolean());

      if (usePredefinedJointAngles.getValue())
      {
         String presetName = jsonNode.get("preset").textValue();
         preset.setValue(presetName.equals(CUSTOM_ANGLES_NAME) ? null : PresetArmConfiguration.valueOf(presetName));
         if (preset.getValue() == null)
         {
            for (int i = 0; i < MAX_NUMBER_OF_JOINTS; i++)
            {
               jointAngles.getValue()[i] = jsonNode.get("j" + i).asDouble();
            }
         }
      }
      else
      {
         palmParentFrameName.setValue(jsonNode.get("parentFrame").textValue());
         JSONTools.toEuclid(jsonNode, palmTransformToParent.getValue());
         holdPoseInWorldLater.setValue(jsonNode.get("holdPoseInWorldLater").asBoolean());
         jointspaceOnly.setValue(jsonNode.get("jointspaceOnly").asBoolean());
         linearPositionWeight.setValue(jsonNode.get("linearPositionWeight").asDouble());
         angularPositionWeight.setValue(jsonNode.get("angularPositionWeight").asDouble());
         positionErrorTolerance.setValue(jsonNode.get("positionErrorTolerance").asDouble());
         orientationErrorTolerance.setValue(Math.toRadians(jsonNode.get("orientationErrorToleranceDegrees").asDouble()));
      }

      jointspaceWeight.setValue(jsonNode.get("jointspaceWeight").asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      onDiskHoldPoseInWorldLater = holdPoseInWorldLater.getValue();
      onDiskJointspaceOnly = jointspaceOnly.getValue();
      onDiskUsePredefinedJointAngles = usePredefinedJointAngles.getValue();
      onDiskPreset = preset.getValue();
      for (int i = 0; i < jointAngles.getLength(); i++)
         onDiskJointAngles[i] = jointAngles.getValueReadOnly(i);
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
      usePredefinedJointAngles.setValue(onDiskUsePredefinedJointAngles);
      preset.setValue(onDiskPreset);
      for (int i = 0; i < jointAngles.getLength(); i++)
         jointAngles.getValue()[i] = onDiskJointAngles[i];
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
      unchanged &= usePredefinedJointAngles.getValue() == onDiskUsePredefinedJointAngles;

      if (usePredefinedJointAngles.getValue()) // Only mark changed for relevant fields
      {
         unchanged &= preset.getValue() == onDiskPreset;
         if (preset.getValue() == null)
            for (int i = 0; i < jointAngles.getLength(); i++)
               unchanged &= jointAngles.getValueReadOnly(i) == onDiskJointAngles[i];
      }
      else
      {
         unchanged &= holdPoseInWorldLater.getValue() == onDiskHoldPoseInWorldLater;
         unchanged &= jointspaceOnly.getValue() == onDiskJointspaceOnly;
         unchanged &= palmParentFrameName.getValue().equals(onDiskPalmParentFrameName);
         unchanged &= palmTransformToParent.getValueReadOnly().equals(onDiskPalmTransformToParent);
         unchanged &= linearPositionWeight.getValue() == onDiskLinearPositionWeight;
         unchanged &= angularPositionWeight.getValue() == onDiskAngularPositionWeight;
         unchanged &= positionErrorTolerance.getValue() == onDiskPositionErrorTolerance;
         unchanged &= orientationErrorTolerance.getValue() == onDiskOrientationErrorTolerance;
      }

      unchanged &= jointspaceWeight.getValue() == onDiskJointspaceWeight;

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
      message.setUsePredefinedJointAngles(usePredefinedJointAngles.toMessage());
      message.setPreset(preset.toMessageOrdinal());
      jointAngles.toMessage(message.getJointAngles());
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
      usePredefinedJointAngles.fromMessage(message.getUsePredefinedJointAngles());
      preset.fromMessageOrdinal(message.getPreset(), PresetArmConfiguration.values);
      jointAngles.fromMessage(message.getJointAngles());
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

   public boolean getUsePredefinedJointAngles()
   {
      return usePredefinedJointAngles.getValue();
   }

   public void setUsePredefinedJointAngles(boolean usePredefinedJointAngles)
   {
      this.usePredefinedJointAngles.setValue(usePredefinedJointAngles);
   }

   @Nullable
   public PresetArmConfiguration getPreset()
   {
      return preset.getValue();
   }

   public void setPreset(@Nullable PresetArmConfiguration preset)
   {
      this.preset.setValue(preset);
   }

   public CRDTUnidirectionalDoubleArray getJointAngles()
   {
      return jointAngles;
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
