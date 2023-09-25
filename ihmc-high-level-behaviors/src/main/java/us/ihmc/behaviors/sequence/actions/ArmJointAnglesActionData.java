package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;

public class ArmJointAnglesActionData implements BehaviorActionData
{
   public static final int NUMBER_OF_JOINTS = 7;
   public static final String CUSTOM_ANGLES_NAME = "CUSTOM_ANGLES";

   private String description = "Arm joint angles";
   @Nullable // Preset is null when using explicitly specified custom joint angles
   private PresetArmConfiguration preset = PresetArmConfiguration.HOME;
   private final double[] jointAngles = new double[NUMBER_OF_JOINTS];
   private RobotSide side = RobotSide.LEFT;
   private double trajectoryDuration = 4.0;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("preset", preset == null ? CUSTOM_ANGLES_NAME : preset.name());
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      if (preset == null)
      {
         for (int i = 0; i < NUMBER_OF_JOINTS; i++)
         {
            jsonNode.put("j" + i, jointAngles[i]);
         }
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      String presetName = jsonNode.get("preset").textValue();
      preset = presetName.equals(CUSTOM_ANGLES_NAME) ? null : PresetArmConfiguration.valueOf(presetName);
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      if (preset == null)
      {
         for (int i = 0; i < NUMBER_OF_JOINTS; i++)
         {
            jointAngles[i] = jsonNode.get("j" + i).asDouble();
         }
      }
   }

   public void toMessage(ArmJointAnglesActionMessage message)
   {
      message.setPreset(preset == null ? -1 : preset.ordinal());
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      for (int i = 0; i < NUMBER_OF_JOINTS; i++)
      {
         message.getJointAngles()[i] = jointAngles[i];
      }
   }

   public void fromMessage(ArmJointAnglesActionMessage message)
   {
      int presetOrdinal = message.getPreset();
      preset = presetOrdinal == -1 ? null : PresetArmConfiguration.values()[presetOrdinal];
      side = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
      for (int i = 0; i < NUMBER_OF_JOINTS; i++)
      {
         jointAngles[i] = message.getJointAngles()[i];
      }
   }

   public double[] getJointAngles()
   {
      return jointAngles;
   }

   @Nullable
   public PresetArmConfiguration getPreset()
   {
      return preset;
   }

   public void setPreset(@Nullable PresetArmConfiguration preset)
   {
      this.preset = preset;
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public RobotSide getSide()
   {
      return side;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
   }

   @Override
   public void setDescription(String description)
   {
      this.description = description;
   }

   @Override
   public String getDescription()
   {
      return description;
   }
}
