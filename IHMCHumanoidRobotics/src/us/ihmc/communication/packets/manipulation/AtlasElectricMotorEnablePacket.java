package us.ihmc.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.tools.random.RandomTools;

/**
 * Created by dstephen on 3/26/15.
 */
@ClassDocumentation("Specifies a specific electric motor in the Atlas forearm to power on or off.")
public class AtlasElectricMotorEnablePacket extends IHMCRosApiPacket<AtlasElectricMotorEnablePacket>
{
   public AtlasElectricMotorPacketEnum motorEnableEnum;
   public boolean enable;

   public AtlasElectricMotorEnablePacket()
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
   }

   public AtlasElectricMotorEnablePacket(Random random)
   {
      this(RandomTools.generateRandomEnum(random, AtlasElectricMotorPacketEnum.class), random.nextBoolean());
   }

   public AtlasElectricMotorEnablePacket(AtlasElectricMotorPacketEnum motorEnableEnum, boolean enable)
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
      this.motorEnableEnum = motorEnableEnum;
      this.enable = enable;
   }

   public AtlasElectricMotorPacketEnum getMotorEnableEnum()
   {
      return motorEnableEnum;
   }

   public boolean isEnabled()
   {
      return enable;
   }

   @Override public boolean epsilonEquals(AtlasElectricMotorEnablePacket other, double epsilon)
   {
      return (this.motorEnableEnum == other.motorEnableEnum) && (this.enable == other.enable);
   }
}
