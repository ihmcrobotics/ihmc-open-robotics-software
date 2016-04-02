package us.ihmc.humanoidRobotics.communication.util;

import javax.vecmath.Quat4d;

import us.ihmc.humanoidRobotics.communication.packets.walking.ChestOrientationPacket;

/**
 * The purpose of this class is to provide a tool to easily create packets
 * for the controller
 */
public class PacketControllerTools
{
   public static ChestOrientationPacket createChestOrientationPacket(Quat4d orientation, double trajectoryTime)
   {
      return new ChestOrientationPacket(orientation, false, trajectoryTime);
   }

   public static ChestOrientationPacket createGoToHomeChestOrientationPacket(double trajectoryTime)
   {
      return new ChestOrientationPacket(null, true, trajectoryTime);
   }

}
