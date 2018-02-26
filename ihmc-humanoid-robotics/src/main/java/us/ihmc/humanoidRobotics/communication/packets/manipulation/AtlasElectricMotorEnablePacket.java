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
   @RosExportedField(documentation = "The Enum value of the motor to enable")
   public AtlasElectricMotorPacketEnum motorEnableEnum;

   @RosExportedField(documentation = "Boolean for enable state; true for enable, false for disable.")
   public boolean enable;

   public AtlasElectricMotorEnablePacket()
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
   }

   public AtlasElectricMotorPacketEnum getMotorEnableEnum()
   {
      return motorEnableEnum;
   }

   @Override
   public void set(AtlasElectricMotorEnablePacket other)
   {
      setPacketInformation(other);
      motorEnableEnum = other.motorEnableEnum;
      enable = other.enable;
   }

   public boolean isEnabled()
   {
      return enable;
   }

   @Override
   public boolean epsilonEquals(AtlasElectricMotorEnablePacket other, double epsilon)
   {
      return (this.motorEnableEnum == other.motorEnableEnum) && (this.enable == other.enable);
   }
}
