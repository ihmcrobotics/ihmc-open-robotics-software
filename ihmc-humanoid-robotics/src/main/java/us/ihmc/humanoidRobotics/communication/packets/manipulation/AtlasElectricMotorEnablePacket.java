package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

/**
 * Created by dstephen on 3/26/15.
 */
@RosMessagePacket(documentation = "Specifies a specific electric motor in the Atlas forearm to power on or off.", rosPackage = "ihmc_atlas_ros", topic = "/control/enable_electric_motor")
public class AtlasElectricMotorEnablePacket extends Packet<AtlasElectricMotorEnablePacket>
{
   public static final byte L_ARM_WRY = 0;
   public static final byte L_ARM_WRX = 1;
   public static final byte L_ARM_WRY2 = 2;
   public static final byte R_ARM_WRY = 3;
   public static final byte R_ARM_WRX = 4;
   public static final byte R_ARM_WRY2 = 5;

   @RosExportedField(documentation = "The Enum value of the motor to enable")
   public byte atlasElectricMotorPacketEnumEnable;

   @RosExportedField(documentation = "Boolean for enable state; true for enable, false for disable.")
   public boolean enable;

   public AtlasElectricMotorEnablePacket()
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
   }

   public byte getAtlasElectricMotorPacketEnumEnable()
   {
      return atlasElectricMotorPacketEnumEnable;
   }

   @Override
   public void set(AtlasElectricMotorEnablePacket other)
   {
      setPacketInformation(other);
      atlasElectricMotorPacketEnumEnable = other.atlasElectricMotorPacketEnumEnable;
      enable = other.enable;
   }

   public boolean getEnable()
   {
      return enable;
   }

   @Override
   public boolean epsilonEquals(AtlasElectricMotorEnablePacket other, double epsilon)
   {
      return (this.atlasElectricMotorPacketEnumEnable == other.atlasElectricMotorPacketEnumEnable) && (this.enable == other.enable);
   }
}
