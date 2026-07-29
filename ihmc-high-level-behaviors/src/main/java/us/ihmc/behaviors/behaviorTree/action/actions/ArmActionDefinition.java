package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.ArmActionDefinitionMessage;
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
   private final CRDTBidirectionalBoolean definedInJointspace;
   private final CRDTBidirectionalEnumField<ArmActionTaskspaceTrajectoryMode> taskspaceTrajectoryMode;
   private final CRDTBidirectionalDouble trajectoryDuration;
   /** Preset is null when using explicitly specified custom joint angles */
   private final CRDTBidirectionalEnumField<PresetArmConfiguration> preset;
   private final CRDTBidirectionalDoubleArray jointAngles;
   private final CRDTBidirectionalString palmParentFrameName;
   private final CRDTBidirectionalRigidBodyTransform palmTransformToParent;
   private final CRDTBidirectionalBoolean jointspaceOnly;
   private final CRDTBidirectionalBoolean holdPoseInWorldLater;
   private final ScrewPrimitiveDefinition screwPrimitive;
   private final CRDTBidirectionalDouble linearPositionWeight;
   private final CRDTBidirectionalDouble angularPositionWeight;
   private final CRDTBidirectionalDouble jointspaceWeight;
   private final CRDTBidirectionalDouble positionErrorTolerance;
   private final CRDTBidirectionalDouble orientationErrorTolerance;

   private RobotSide onDiskSide;
   private boolean onDiskDefinedInJointspace;
   private ArmActionTaskspaceTrajectoryMode onDiskTaskspaceTrajectoryMode;
   private double onDiskTrajectoryDuration;
   private PresetArmConfiguration onDiskPreset;
   private final double[] onDiskJointAngles = new double[MAX_NUMBER_OF_JOINTS];
   private String onDiskPalmParentFrameName;
   private final RigidBodyTransform onDiskPalmTransformToParent = new RigidBodyTransform();
   private boolean onDiskJointspaceOnly;
   private boolean onDiskHoldPoseInWorldLater;
   private double onDiskLinearPositionWeight;
   private double onDiskAngularPositionWeight;
   private double onDiskJointspaceWeight;
   private double onDiskPositionErrorTolerance;
   private double onDiskOrientationErrorTolerance;

   public ArmActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      side = new CRDTBidirectionalEnumField<>(this, RobotSide.LEFT);
      definedInJointspace = new CRDTBidirectionalBoolean(this, DEFAULT_DEFINED_IN_JOINTSPACE);
      taskspaceTrajectoryMode = new CRDTBidirectionalEnumField<>(this, ArmActionTaskspaceTrajectoryMode.SINGLE_POSE);
      trajectoryDuration = new CRDTBidirectionalDouble(this, DEFAULT_TRAJECTORY_DURATION);
      preset = new CRDTBidirectionalEnumField<>(this, PresetArmConfiguration.HOME);
      jointAngles = new CRDTBidirectionalDoubleArray(this, MAX_NUMBER_OF_JOINTS);
      palmParentFrameName = new CRDTBidirectionalString(this, "Chest");
      palmTransformToParent = new CRDTBidirectionalRigidBodyTransform(this);
      jointspaceOnly = new CRDTBidirectionalBoolean(this, DEFAULT_IS_JOINTSPACE_MODE);
      holdPoseInWorldLater = new CRDTBidirectionalBoolean(this, DEFAULT_HOLD_POSE);
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
         jsonNode.put("jointspaceWeight", jointspaceWeight.getValue());
         jsonNode.put("positionErrorTolerance", Double.parseDouble("%.3f".formatted(positionErrorTolerance.getValue())));
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
            jsonNode.put("jointspaceWeight", jointspaceWeight.getValue());
            jsonNode.put("positionErrorTolerance", Double.parseDouble("%.3f".formatted(positionErrorTolerance.getValue())));
            jsonNode.put("orientationErrorToleranceDegrees", Double.parseDouble("%.3f".formatted(Math.toDegrees(orientationErrorTolerance.getValue()))));
         }
         else
         {
            screwPrimitive.saveToFile(jsonNode);
            jsonNode.put("linearPositionWeight", linearPositionWeight.getValue());
            jsonNode.put("angularPositionWeight", angularPositionWeight.getValue());
            jsonNode.put("jointspaceWeight", jointspaceWeight.getValue());
            jsonNode.put("positionErrorTolerance", Double.parseDouble("%.3f".formatted(positionErrorTolerance.getValue())));
            jsonNode.put("orientationErrorToleranceDegrees", Double.parseDouble("%.3f".formatted(Math.toDegrees(orientationErrorTolerance.getValue()))));
         }
      }
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
         jointspaceWeight.setValue(jsonNode.get("jointspaceWeight").asDouble());
         if (jsonNode.get("positionErrorTolerance") instanceof NumericNode node)
            positionErrorTolerance.setValue(node.asDouble());
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
            jointspaceOnly.setValue(jsonNode.get("jointspaceOnly").asBoolean());
            holdPoseInWorldLater.setValue(jsonNode.get("holdPoseInWorldLater").asBoolean());
            linearPositionWeight.setValue(jsonNode.get("linearPositionWeight").asDouble());
            angularPositionWeight.setValue(jsonNode.get("angularPositionWeight").asDouble());
            jointspaceWeight.setValue(jsonNode.get("jointspaceWeight").asDouble());
            if (jsonNode.get("positionErrorTolerance") instanceof NumericNode node)
               positionErrorTolerance.setValue(node.asDouble());
            orientationErrorTolerance.setValue(Math.toRadians(jsonNode.get("orientationErrorToleranceDegrees").asDouble()));
         }
         else
         {
            screwPrimitive.loadFromFile(jsonNode);
            if (jsonNode.has("jointspaceOnly"))
               jointspaceOnly.setValue(jsonNode.get("jointspaceOnly").asBoolean());
            if (jsonNode.has("linearPositionWeight"))
               linearPositionWeight.setValue(jsonNode.get("linearPositionWeight").asDouble());
            if (jsonNode.has("angularPositionWeight"))
               angularPositionWeight.setValue(jsonNode.get("angularPositionWeight").asDouble());
            if (jsonNode.has("jointspaceWeight"))
               jointspaceWeight.setValue(jsonNode.get("jointspaceWeight").asDouble());
            if (jsonNode.get("positionErrorTolerance") instanceof NumericNode node)
               positionErrorTolerance.setValue(node.asDouble());
            if (jsonNode.has("orientationErrorToleranceDegrees"))
               orientationErrorTolerance.setValue(Math.toRadians(jsonNode.get("orientationErrorToleranceDegrees").asDouble()));
         }
      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskSide = side.getValue();
      onDiskDefinedInJointspace = definedInJointspace.getValue();
      onDiskTaskspaceTrajectoryMode = taskspaceTrajectoryMode.getValue();
      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      onDiskPreset = preset.getValue();
      for (int i = 0; i < jointAngles.getLength(); i++)
         onDiskJointAngles[i] = jointAngles.getValueReadOnly(i);
      onDiskPalmParentFrameName = palmParentFrameName.getValue();
      onDiskPalmTransformToParent.set(palmTransformToParent.getValueReadOnly());
      onDiskJointspaceOnly = jointspaceOnly.getValue();
      onDiskHoldPoseInWorldLater = holdPoseInWorldLater.getValue();
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
         definedInJointspace.setValue(onDiskDefinedInJointspace);
         taskspaceTrajectoryMode.setValue(onDiskTaskspaceTrajectoryMode);
         trajectoryDuration.setValue(onDiskTrajectoryDuration);
         preset.setValue(onDiskPreset);
         for (int i = 0; i < jointAngles.getLength(); i++)
            jointAngles.setValue(i, onDiskJointAngles[i]);
         palmParentFrameName.setValue(onDiskPalmParentFrameName);
         palmTransformToParent.getValueAndModify().set(onDiskPalmTransformToParent);
         jointspaceOnly.setValue(onDiskJointspaceOnly);
         holdPoseInWorldLater.setValue(onDiskHoldPoseInWorldLater);
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
         unchanged &= jointspaceWeight.getValue() == onDiskJointspaceWeight;
         unchanged &= positionErrorTolerance.getValue() == onDiskPositionErrorTolerance;
      }
      else
      {
         unchanged &= taskspaceTrajectoryMode.getValue() == onDiskTaskspaceTrajectoryMode;
         unchanged &= palmParentFrameName.getValue().equals(onDiskPalmParentFrameName);

         if (taskspaceTrajectoryMode.getValue() == ArmActionTaskspaceTrajectoryMode.SINGLE_POSE)
         {
            unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
            unchanged &= palmTransformToParent.getValueReadOnly().equals(onDiskPalmTransformToParent);
            unchanged &= jointspaceOnly.getValue() == onDiskJointspaceOnly;
            unchanged &= holdPoseInWorldLater.getValue() == onDiskHoldPoseInWorldLater;
            unchanged &= linearPositionWeight.getValue() == onDiskLinearPositionWeight;
            unchanged &= angularPositionWeight.getValue() == onDiskAngularPositionWeight;
            unchanged &= jointspaceWeight.getValue() == onDiskJointspaceWeight;
            unchanged &= positionErrorTolerance.getValue() == onDiskPositionErrorTolerance;
            unchanged &= orientationErrorTolerance.getValue() == onDiskOrientationErrorTolerance;
         }
         else
         {
            unchanged &= !screwPrimitive.hasChanges();
            unchanged &= linearPositionWeight.getValue() == onDiskLinearPositionWeight;
            unchanged &= angularPositionWeight.getValue() == onDiskAngularPositionWeight;
            unchanged &= jointspaceWeight.getValue() == onDiskJointspaceWeight;
            unchanged &= positionErrorTolerance.getValue() == onDiskPositionErrorTolerance;
            unchanged &= orientationErrorTolerance.getValue() == onDiskOrientationErrorTolerance;
         }
      }

      return !unchanged;
   }

   public void toMessage(ArmActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setDefinedInJointspace(definedInJointspace.toMessage());
      message.setTaskspaceTrajectoryMode((byte) taskspaceTrajectoryMode.getValue().ordinal());
      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      message.setPreset(preset.toMessageOrdinal());
      jointAngles.toMessage(message.getJointAngles());
      message.setParentFrameName(palmParentFrameName.toMessage());
      palmTransformToParent.toMessage(message.getTransformToParent());
      message.setJointSpaceControl(jointspaceOnly.toMessage());
      message.setHoldPoseInWorld(holdPoseInWorldLater.toMessage());
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

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      definedInJointspace.fromMessage(message.getDefinedInJointspace());
      taskspaceTrajectoryMode.fromMessageOrdinal(message.getTaskspaceTrajectoryMode(), ArmActionTaskspaceTrajectoryMode.values);
      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      preset.fromMessageOrdinal(message.getPreset(), PresetArmConfiguration.values);
      jointAngles.fromMessage(message.getJointAngles());
      palmParentFrameName.fromMessage(message.getParentFrameNameAsString());
      palmTransformToParent.fromMessage(message.getTransformToParent());
      jointspaceOnly.fromMessage(message.getJointSpaceControl());
      holdPoseInWorldLater.fromMessage(message.getHoldPoseInWorld());
      screwPrimitive.fromMessage(message);
      linearPositionWeight.fromMessage(message.getLinearPositionWeight());
      angularPositionWeight.fromMessage(message.getAngularPositionWeight());
      jointspaceWeight.fromMessage(message.getJointspaceWeight());
      positionErrorTolerance.fromMessage(message.getPositionErrorTolerance());
      orientationErrorTolerance.fromMessage(message.getOrientationErrorTolerance());
   }

   public ScrewPrimitiveDefinition getScrewPrimitive()
   {
      return screwPrimitive;
   }

   public CRDTBidirectionalString getCRDTPalmParentFrameName()
   {
      return palmParentFrameName;
   }

   public CRDTBidirectionalDoubleArray getJointAngles()
   {
      return jointAngles;
   }

   public CRDTBidirectionalRigidBodyTransform getPalmTransformToParent()
   {
      return palmTransformToParent;
   }

   public CRDTBidirectionalRigidBodyTransform getScrewAxisPoseInObjectFrame()
   {
      return screwPrimitive.getScrewAxisPoseInObjectFrame();
   }

   public void setSide(RobotSide side)
   {
      this.side.setValue(side);
   }

   @Override
   public RobotSide getSide()
   {
      return side.getValue();
   }

   public void setDefinedInJointspace(boolean definedInJointspace)
   {
      this.definedInJointspace.setValue(definedInJointspace);
   }

   public boolean getDefinedInJointspace()
   {
      return definedInJointspace.getValue();
   }

   public void setTaskspaceTrajectoryMode(ArmActionTaskspaceTrajectoryMode taskspaceTrajectoryMode)
   {
      this.taskspaceTrajectoryMode.setValue(taskspaceTrajectoryMode);
   }

   public ArmActionTaskspaceTrajectoryMode getTaskspaceTrajectoryMode()
   {
      return taskspaceTrajectoryMode.getValue();
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration.setValue(trajectoryDuration);
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration.getValue();
   }

   public void setPreset(@Nullable PresetArmConfiguration preset)
   {
      this.preset.setValue(preset);
   }

   @Nullable
   public PresetArmConfiguration getPreset()
   {
      return preset.getValue();
   }

   public void setPalmParentFrameName(String palmParentFrameName)
   {
      this.palmParentFrameName.setValue(palmParentFrameName);
   }

   public String getPalmParentFrameName()
   {
      return palmParentFrameName.getValue();
   }

   public void setJointspaceOnly(boolean jointspaceOnly)
   {
      this.jointspaceOnly.setValue(jointspaceOnly);
   }

   public boolean getJointspaceOnly()
   {
      return jointspaceOnly.getValue();
   }

   public void setHoldPoseInWorldLater(boolean holdPoseInWorldLater)
   {
      this.holdPoseInWorldLater.setValue(holdPoseInWorldLater);
   }

   public boolean getHoldPoseInWorldLater()
   {
      return holdPoseInWorldLater.getValue();
   }

   public void setTranslation(double translation)
   {
      screwPrimitive.setTranslation(translation);
   }

   public double getTranslation()
   {
      return screwPrimitive.getTranslation();
   }

   public void setRotation(double rotation)
   {
      screwPrimitive.setRotation(rotation);
   }

   public double getRotation()
   {
      return screwPrimitive.getRotation();
   }

   public void setMaxLinearVelocity(double maxLinearVelocity)
   {
      screwPrimitive.setMaxLinearVelocity(maxLinearVelocity);
   }

   public double getMaxLinearVelocity()
   {
      return screwPrimitive.getMaxLinearVelocity();
   }

   public void setMaxAngularVelocity(double maxAngularVelocity)
   {
      screwPrimitive.setMaxAngularVelocity(maxAngularVelocity);
   }

   public double getMaxAngularVelocity()
   {
      return screwPrimitive.getMaxAngularVelocity();
   }

   public void setLinearPositionWeight(double linearPositionWeight)
   {
      this.linearPositionWeight.setValue(linearPositionWeight);
   }

   public double getLinearPositionWeight()
   {
      return linearPositionWeight.getValue();
   }

   public void setAngularPositionWeight(double angularPositionWeight)
   {
      this.angularPositionWeight.setValue(angularPositionWeight);
   }

   public double getAngularPositionWeight()
   {
      return angularPositionWeight.getValue();
   }

   public void setJointspaceWeight(double jointspaceWeight)
   {
      this.jointspaceWeight.setValue(jointspaceWeight);
   }

   public double getJointspaceWeight()
   {
      return jointspaceWeight.getValue();
   }

   public void setPositionErrorTolerance(double positionErrorTolerance)
   {
      this.positionErrorTolerance.setValue(positionErrorTolerance);
   }

   public double getPositionErrorTolerance()
   {
      return positionErrorTolerance.getValue();
   }

   public void setOrientationErrorTolerance(double orientationErrorTolerance)
   {
      this.orientationErrorTolerance.setValue(orientationErrorTolerance);
   }

   public double getOrientationErrorTolerance()
   {
      return orientationErrorTolerance.getValue();
   }
}
