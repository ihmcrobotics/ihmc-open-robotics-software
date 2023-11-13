package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;

public class ArmJointAnglesActionDefinition extends BehaviorActionDefinition
{
   public static final int NUMBER_OF_JOINTS = 7;
   public static final String CUSTOM_ANGLES_NAME = "CUSTOM_ANGLES";

   @Nullable // Preset is null when using explicitly specified custom joint angles
   private PresetArmConfiguration preset = PresetArmConfiguration.HOME;
   private RobotSide side = RobotSide.LEFT;
   private double trajectoryDuration = 4.0;
   private double[] jointAngles = new double[NUMBER_OF_JOINTS];

   public ArmJointAnglesActionDefinition()
   {
      super("Arm joint angles");
   }
   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

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
      super.loadFromFile(jsonNode);

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

   public void toMessage(ArmJointAnglesActionDefinitionMessage message)
   {
      super.toMessage(message.getActionDefinition());

      message.setPreset(preset == null ? -1 : preset.ordinal());
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      for (int i = 0; i < NUMBER_OF_JOINTS; i++)
      {
         message.getJointAngles()[i] = jointAngles[i];
      }
   }

   public void fromMessage(ArmJointAnglesActionDefinitionMessage message)
   {
      super.fromMessage(message.getActionDefinition());

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

   public void setJointAngles(double[] jointAngles)
   {
      this.jointAngles = jointAngles;
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
}
