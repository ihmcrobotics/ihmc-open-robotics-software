package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.ArmActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.NumericNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;

import javax.annotation.Nullable;

public class ArmActionDefinition extends ActionNodeDefinition implements SidedObject
{
   public static final double DEFAULT_TRAJECTORY_DURATION = 4.0;
   public static final boolean DEFAULT_IS_JOINTSPACE_MODE = true;
   public static final boolean DEFAULT_DEFINED_IN_JOINTSPACE = false;
   public static final int MAX_NUMBER_OF_JOINTS = 7;
   public static final String CUSTOM_ANGLES_NAME = "CUSTOM_ANGLES";
   public static final boolean DEFAULT_HOLD_POSE = false;
   public static final double DEFAULT_LINEAR_POSITION_WEIGHT = 50.0;
   public static final double DEFAULT_ANGULAR_POSITION_WEIGHT = 50.0;
   public static final double DEFAULT_JOINTSPACE_WEIGHT = -1.0;
   public static final double DEFAULT_POSITION_ERROR_TOLERANCE = 0.3;
   public static final double DEFAULT_ORIENTATION_ERROR_TOLERANCE = Math.toRadians(20.0);

   private final CRDTBidirectionalEnumField<RobotSide> side;
   private final CRDTBidirectionalDouble trajectoryDuration;
   private final CRDTBidirectionalBoolean holdPoseInWorldLater;
   private final CRDTBidirectionalBoolean jointspaceOnly;
   private final CRDTBidirectionalBoolean definedInJointspace;
   private final CRDTBidirectionalEnumField<ArmActionTaskspaceTrajectoryMode> taskspaceTrajectoryMode;
   /** Preset is null when using explicitly specified custom joint angles */
   private final CRDTBidirectionalEnumField<PresetArmConfiguration> preset;
   private final CRDTBidirectionalDoubleArray jointAngles;
   private final CRDTBidirectionalString palmParentFrameName;
   private final CRDTBidirectionalRigidBodyTransform palmTransformToParent;
   private final ScrewPrimitiveDefinition screwPrimitive;
   private final CRDTBidirectionalDouble linearPositionWeight;
   private final CRDTBidirectionalDouble angularPositionWeight;
   private final CRDTBidirectionalDouble jointspaceWeight;
   private final CRDTBidirectionalDouble positionErrorTolerance;
   private final CRDTBidirectionalDouble orientationErrorTolerance;

   // On disk fields
   private RobotSide onDiskSide;
   private double onDiskTrajectoryDuration;
   private boolean onDiskHoldPoseInWorldLater;
   private boolean onDiskJointspaceOnly;
   private boolean onDiskDefinedInJointspace;
   private ArmActionTaskspaceTrajectoryMode onDiskTaskspaceTrajectoryMode;
   private PresetArmConfiguration onDiskPreset;
   private final double[] onDiskJointAngles = new double[MAX_NUMBER_OF_JOINTS];
   private String onDiskPalmParentFrameName;
   private final RigidBodyTransform onDiskPalmTransformToParent = new RigidBodyTransform();
   private double onDiskLinearPositionWeight;
   private double onDiskAngularPositionWeight;
   private double onDiskJointspaceWeight;
   private double onDiskPositionErrorTolerance;
   private double onDiskOrientationErrorTolerance;

   public ArmActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      side = new CRDTBidirectionalEnumField<>(this, RobotSide.LEFT);
      trajectoryDuration = new CRDTBidirectionalDouble(this, DEFAULT_TRAJECTORY_DURATION);
      holdPoseInWorldLater = new CRDTBidirectionalBoolean(this, DEFAULT_HOLD_POSE);
      jointspaceOnly = new CRDTBidirectionalBoolean(this, DEFAULT_IS_JOINTSPACE_MODE);
      definedInJointspace = new CRDTBidirectionalBoolean(this, DEFAULT_DEFINED_IN_JOINTSPACE);
      taskspaceTrajectoryMode = new CRDTBidirectionalEnumField<>(this, ArmActionTaskspaceTrajectoryMode.SINGLE_POSE);
      preset = new CRDTBidirectionalEnumField<>(this, PresetArmConfiguration.HOME);
      jointAngles = new CRDTBidirectionalDoubleArray(this, MAX_NUMBER_OF_JOINTS);
      palmParentFrameName = new CRDTBidirectionalString(this, "Chest");
      palmTransformToParent = new CRDTBidirectionalRigidBodyTransform(this);
      screwPrimitive = new ScrewPrimitiveDefinition(this);
      linearPositionWeight = new CRDTBidirectionalDouble(this, DEFAULT_LINEAR_POSITION_WEIGHT);
      angularPositionWeight = new CRDTBidirectionalDouble(this, DEFAULT_ANGULAR_POSITION_WEIGHT);
      jointspaceWeight = new CRDTBidirectionalDouble(this, DEFAULT_JOINTSPACE_WEIGHT);
      positionErrorTolerance = new CRDTBidirectionalDouble(this, DEFAULT_POSITION_ERROR_TOLERANCE);
      orientationErrorTolerance = new CRDTBidirectionalDouble(this, DEFAULT_ORIENTATION_ERROR_TOLERANCE);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("definedInJointspace", definedInJointspace.getValue());

      if (definedInJointspace.getValue())
      {
         jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
         jsonNode.put("preset", preset.getValue() == null ? CUSTOM_ANGLES_NAME : preset.getValue().name());
         if (preset.getValue() == null)
            for (int i = 0; i < MAX_NUMBER_OF_JOINTS; i++)
               jsonNode.put("j" + i + "Degrees", JSONTools.toJsonRadians(jointAngles.getValueReadOnly(i)));
      }
      else
      {
         jsonNode.put("taskspaceTrajectoryMode", taskspaceTrajectoryMode.getValue().name());
         jsonNode.put("parentFrame", palmParentFrameName.getValue());

         if (taskspaceTrajectoryMode.getValue() == ArmActionTaskspaceTrajectoryMode.SINGLE_POSE)
         {
            jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
            JSONTools.toJSON(jsonNode, palmTransformToParent.getValueReadOnly());
            jsonNode.put("jointspaceOnly", jointspaceOnly.getValue());
            jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater.getValue());
            jsonNode.put("linearPositionWeight", linearPositionWeight.getValue());
            jsonNode.put("angularPositionWeight", angularPositionWeight.getValue());
            jsonNode.put("orientationErrorToleranceDegrees", Double.parseDouble("%.3f".formatted(Math.toDegrees(orientationErrorTolerance.getValue()))));
         }
         else
         {
            screwPrimitive.saveToFile(jsonNode, Double.parseDouble("%.3f".formatted(Math.toDegrees(orientationErrorTolerance.getValue()))));
         }
      }

      jsonNode.put("positionErrorTolerance", Double.parseDouble("%.3f".formatted(positionErrorTolerance.getValue())));
      jsonNode.put("jointspaceWeight", jointspaceWeight.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));

      if (jsonNode.has("definedInJointspace"))
         definedInJointspace.setValue(jsonNode.get("definedInJointspace").asBoolean());
      else if (jsonNode.has("usePredefinedJointAngles"))
         definedInJointspace.setValue(jsonNode.get("usePredefinedJointAngles").asBoolean());
      else
         definedInJointspace.setValue(DEFAULT_DEFINED_IN_JOINTSPACE);

      if (definedInJointspace.getValue())
      {
         trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
         String presetName = jsonNode.get("preset").textValue();
         preset.setValue(presetName.equals(CUSTOM_ANGLES_NAME) ? null : PresetArmConfiguration.valueOf(presetName));
         if (preset.getValue() == null)
            for (int i = 0; i < MAX_NUMBER_OF_JOINTS; i++)
               jointAngles.setValue(i, Math.toRadians(jsonNode.get("j" + i + "Degrees").asDouble()));
      }
      else
      {
         if (jsonNode.has("taskspaceTrajectoryMode"))
            taskspaceTrajectoryMode.setValue(ArmActionTaskspaceTrajectoryMode.valueOf(jsonNode.get("taskspaceTrajectoryMode").asText()));
         else
            taskspaceTrajectoryMode.setValue(ArmActionTaskspaceTrajectoryMode.SINGLE_POSE);

         if (jsonNode.has("parentFrame"))
            palmParentFrameName.setValue(jsonNode.get("parentFrame").textValue());
         else if (jsonNode.has("objectFrame"))
            palmParentFrameName.setValue(jsonNode.get("objectFrame").textValue());

         if (taskspaceTrajectoryMode.getValue() == ArmActionTaskspaceTrajectoryMode.SINGLE_POSE)
         {
            trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
            JSONTools.toEuclid(jsonNode, palmTransformToParent.getValueAndModify());
            holdPoseInWorldLater.setValue(jsonNode.get("holdPoseInWorldLater").asBoolean());
            jointspaceOnly.setValue(jsonNode.get("jointspaceOnly").asBoolean());
            linearPositionWeight.setValue(jsonNode.get("linearPositionWeight").asDouble());
            angularPositionWeight.setValue(jsonNode.get("angularPositionWeight").asDouble());
            orientationErrorTolerance.setValue(Math.toRadians(jsonNode.get("orientationErrorToleranceDegrees").asDouble()));
         }
         else
         {
            screwPrimitive.loadFromFile(jsonNode);
            if (jsonNode.has("jointspaceOnly"))
               jointspaceOnly.setValue(jsonNode.get("jointspaceOnly").asBoolean());
            if (jsonNode.has("orientationErrorToleranceDegrees"))
               orientationErrorTolerance.setValue(Math.toRadians(jsonNode.get("orientationErrorToleranceDegrees").asDouble()));
         }
      }

      if (jsonNode.get("positionErrorTolerance") instanceof NumericNode node)
         positionErrorTolerance.setValue(node.asDouble());
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
      onDiskDefinedInJointspace = definedInJointspace.getValue();
      onDiskTaskspaceTrajectoryMode = taskspaceTrajectoryMode.getValue();
      onDiskPreset = preset.getValue();
      for (int i = 0; i < jointAngles.getLength(); i++)
         onDiskJointAngles[i] = jointAngles.getValueReadOnly(i);
      onDiskPalmParentFrameName = palmParentFrameName.getValue();
      onDiskPalmTransformToParent.set(palmTransformToParent.getValueReadOnly());
      screwPrimitive.setOnDiskFields();
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

      if (isUndoAvailable())
      {
         side.setValue(onDiskSide);
         trajectoryDuration.setValue(onDiskTrajectoryDuration);
         holdPoseInWorldLater.setValue(onDiskHoldPoseInWorldLater);
         jointspaceOnly.setValue(onDiskJointspaceOnly);
         definedInJointspace.setValue(onDiskDefinedInJointspace);
         taskspaceTrajectoryMode.setValue(onDiskTaskspaceTrajectoryMode);
         preset.setValue(onDiskPreset);
         for (int i = 0; i < jointAngles.getLength(); i++)
            jointAngles.setValue(i, onDiskJointAngles[i]);
         palmParentFrameName.setValue(onDiskPalmParentFrameName);
         palmTransformToParent.getValueAndModify().set(onDiskPalmTransformToParent);
         screwPrimitive.undoAllNontopologicalChanges();
         linearPositionWeight.setValue(onDiskLinearPositionWeight);
         angularPositionWeight.setValue(onDiskAngularPositionWeight);
         jointspaceWeight.setValue(onDiskJointspaceWeight);
         positionErrorTolerance.setValue(onDiskPositionErrorTolerance);
         orientationErrorTolerance.setValue(onDiskOrientationErrorTolerance);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= side.getValue() == onDiskSide;
      unchanged &= definedInJointspace.getValue() == onDiskDefinedInJointspace;

      if (definedInJointspace.getValue())
      {
         unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
         unchanged &= preset.getValue() == onDiskPreset;
         if (preset.getValue() == null)
            for (int i = 0; i < jointAngles.getLength(); i++)
               unchanged &= jointAngles.getValueReadOnly(i) == onDiskJointAngles[i];
      }
      else
      {
         unchanged &= taskspaceTrajectoryMode.getValue() == onDiskTaskspaceTrajectoryMode;
         unchanged &= palmParentFrameName.getValue().equals(onDiskPalmParentFrameName);

         if (taskspaceTrajectoryMode.getValue() == ArmActionTaskspaceTrajectoryMode.SINGLE_POSE)
         {
            unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
            unchanged &= holdPoseInWorldLater.getValue() == onDiskHoldPoseInWorldLater;
            unchanged &= jointspaceOnly.getValue() == onDiskJointspaceOnly;
            unchanged &= palmTransformToParent.getValueReadOnly().equals(onDiskPalmTransformToParent);
            unchanged &= linearPositionWeight.getValue() == onDiskLinearPositionWeight;
            unchanged &= angularPositionWeight.getValue() == onDiskAngularPositionWeight;
            unchanged &= orientationErrorTolerance.getValue() == onDiskOrientationErrorTolerance;
         }
         else
         {
            unchanged &= !screwPrimitive.hasChanges();
            unchanged &= orientationErrorTolerance.getValue() == onDiskOrientationErrorTolerance;
         }
      }

      unchanged &= positionErrorTolerance.getValue() == onDiskPositionErrorTolerance;
      unchanged &= jointspaceWeight.getValue() == onDiskJointspaceWeight;

      return !unchanged;
   }

   public void toMessage(ArmActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setParentFrameName(palmParentFrameName.toMessage());
      palmTransformToParent.toMessage(message.getTransformToParent());
      message.setRobotSide(side.toMessage().toByte());
      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      message.setHoldPoseInWorld(holdPoseInWorldLater.toMessage());
      message.setJointSpaceControl(jointspaceOnly.toMessage());
      message.setDefinedInJointspace(definedInJointspace.toMessage());
      message.setTaskspaceTrajectoryMode((byte) taskspaceTrajectoryMode.getValue().ordinal());
      message.setPreset(preset.toMessageOrdinal());
      jointAngles.toMessage(message.getJointAngles());
      screwPrimitive.toMessage(message);
      message.setLinearPositionWeight(linearPositionWeight.toMessage());
      message.setAngularPositionWeight(angularPositionWeight.toMessage());
      message.setJointspaceWeight(jointspaceWeight.toMessage());
      message.setPositionErrorTolerance(positionErrorTolerance.toMessage());
      message.setOrientationErrorTolerance(orientationErrorTolerance.toMessage());
   }

   public void fromMessage(ArmActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      palmParentFrameName.fromMessage(message.getParentFrameNameAsString());
      palmTransformToParent.fromMessage(message.getTransformToParent());
      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      holdPoseInWorldLater.fromMessage(message.getHoldPoseInWorld());
      jointspaceOnly.fromMessage(message.getJointSpaceControl());
      definedInJointspace.fromMessage(message.getDefinedInJointspace());
      taskspaceTrajectoryMode.fromMessageOrdinal(message.getTaskspaceTrajectoryMode(), ArmActionTaskspaceTrajectoryMode.values);
      preset.fromMessageOrdinal(message.getPreset(), PresetArmConfiguration.values);
      jointAngles.fromMessage(message.getJointAngles());
      screwPrimitive.fromMessage(message);
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

   public boolean getDefinedInJointspace()
   {
      return definedInJointspace.getValue();
   }

   public void setDefinedInJointspace(boolean definedInJointspace)
   {
      this.definedInJointspace.setValue(definedInJointspace);
   }

   public ArmActionTaskspaceTrajectoryMode getTaskspaceTrajectoryMode()
   {
      return taskspaceTrajectoryMode.getValue();
   }

   public void setTaskspaceTrajectoryMode(ArmActionTaskspaceTrajectoryMode taskspaceTrajectoryMode)
   {
      this.taskspaceTrajectoryMode.setValue(taskspaceTrajectoryMode);
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

   public CRDTBidirectionalDoubleArray getJointAngles()
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

   public CRDTBidirectionalString getCRDTPalmParentFrameName()
   {
      return palmParentFrameName;
   }

   public CRDTBidirectionalRigidBodyTransform getPalmTransformToParent()
   {
      return palmTransformToParent;
   }

   public ScrewPrimitiveDefinition getScrewPrimitive()
   {
      return screwPrimitive;
   }

   public CRDTBidirectionalRigidBodyTransform getScrewAxisPoseInObjectFrame()
   {
      return screwPrimitive.getScrewAxisPoseInObjectFrame();
   }

   public double getTranslation()
   {
      return screwPrimitive.getTranslation();
   }

   public void setTranslation(double translation)
   {
      screwPrimitive.setTranslation(translation);
   }

   public double getRotation()
   {
      return screwPrimitive.getRotation();
   }

   public void setRotation(double rotation)
   {
      screwPrimitive.setRotation(rotation);
   }

   public double getMaxLinearVelocity()
   {
      return screwPrimitive.getMaxLinearVelocity();
   }

   public void setMaxLinearVelocity(double maxLinearVelocity)
   {
      screwPrimitive.setMaxLinearVelocity(maxLinearVelocity);
   }

   public double getMaxAngularVelocity()
   {
      return screwPrimitive.getMaxAngularVelocity();
   }

   public void setMaxAngularVelocity(double maxAngularVelocity)
   {
      screwPrimitive.setMaxAngularVelocity(maxAngularVelocity);
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
