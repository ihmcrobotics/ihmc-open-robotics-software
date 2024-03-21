package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalDoubleArray;
import us.ihmc.communication.crdt.CRDTUnidirectionalEnumField;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

public class ArmJointAnglesActionDefinition extends ActionNodeDefinition
{
   public static final int MAX_NUMBER_OF_JOINTS = 7;
   public static final String CUSTOM_ANGLES_NAME = "CUSTOM_ANGLES";

   /** Preset is null when using explicitly specified custom joint angles */
   private final CRDTUnidirectionalEnumField<PresetArmConfiguration> preset;
   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalDouble trajectoryDuration;
   private final CRDTUnidirectionalDoubleArray jointAngles;

   // On disk fields
   private PresetArmConfiguration onDiskPreset;
   private RobotSide onDiskSide;
   private double onDiskTrajectoryDuration;
   private final double[] onDiskJointAngles = new double[MAX_NUMBER_OF_JOINTS];

   public ArmJointAnglesActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      preset = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, PresetArmConfiguration.HOME);
      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      trajectoryDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 4.0);
      jointAngles = new CRDTUnidirectionalDoubleArray(ROS2ActorDesignation.OPERATOR, crdtInfo, MAX_NUMBER_OF_JOINTS);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("preset", preset.getValue() == null ? CUSTOM_ANGLES_NAME : preset.getValue().name());
      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
      if (preset.getValue() == null)
      {
         for (int i = 0; i < MAX_NUMBER_OF_JOINTS; i++)
         {
            jsonNode.put("j" + i, jointAngles.getValueReadOnly(i));
         }
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      String presetName = jsonNode.get("preset").textValue();
      preset.setValue(presetName.equals(CUSTOM_ANGLES_NAME) ? null : PresetArmConfiguration.valueOf(presetName));
      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
      if (preset.getValue() == null)
      {
         for (int i = 0; i < MAX_NUMBER_OF_JOINTS; i++)
         {
            jointAngles.getValue()[i] = jsonNode.get("j" + i).asDouble();
         }
      }
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskPreset = preset.getValue();
      onDiskSide = side.getValue();
      onDiskTrajectoryDuration = trajectoryDuration.getValue();
      for (int i = 0; i < jointAngles.getLength(); i++)
         onDiskJointAngles[i] = jointAngles.getValueReadOnly(i);
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      preset.setValue(onDiskPreset);
      side.setValue(onDiskSide);
      trajectoryDuration.setValue(onDiskTrajectoryDuration);
      for (int i = 0; i < jointAngles.getLength(); i++)
         jointAngles.getValue()[i] = onDiskJointAngles[i];
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= preset.getValue() == onDiskPreset;
      unchanged &= side.getValue() == onDiskSide;
      unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;
      if (preset.getValue() == null)
         for (int i = 0; i < jointAngles.getLength(); i++)
            unchanged &= jointAngles.getValueReadOnly(i) == onDiskJointAngles[i];

      return !unchanged;
   }

   public void toMessage(ArmJointAnglesActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setPreset(preset.toMessageOrdinal());
      message.setRobotSide(side.toMessage().toByte());
      message.setTrajectoryDuration(trajectoryDuration.toMessage());
      jointAngles.toMessage(message.getJointAngles());
   }

   public void fromMessage(ArmJointAnglesActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      preset.fromMessageOrdinal(message.getPreset(), PresetArmConfiguration.values);
      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
      jointAngles.fromMessage(message.getJointAngles());
   }

   public CRDTUnidirectionalDoubleArray getJointAngles()
   {
      return jointAngles;
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

   public double getTrajectoryDuration()
   {
      return trajectoryDuration.getValue();
   }

   public RobotSide getSide()
   {
      return side.getValue();
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration.setValue(trajectoryDuration);
   }

   public void setSide(RobotSide side)
   {
      this.side.setValue(side);
   }
}
