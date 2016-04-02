package us.ihmc.humanoidRobotics.communication.util;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.humanoidRobotics.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisPosePacket;

/**
 * The purpose of this class is to provide a tool to easily create packets
 * for the controller
 */
public class PacketControllerTools
{
   public static PelvisPosePacket createPelvisPosePacketForPositionAndOrientation(Point3d position, Quat4d orientation, double trajectoryTime)
   {
      return new PelvisPosePacket(position, orientation, false, trajectoryTime);
   }

   public static PelvisPosePacket createPelvisPosePacketForOrientationOnly(Quat4d orientation, double trajectoryTime)
   {
      return new PelvisPosePacket(null, orientation, false, trajectoryTime);
   }

   public static PelvisPosePacket createPelvisPosePacketForPositionOnly(Point3d position, double trajectoryTime)
   {
      return new PelvisPosePacket(position, null, false, trajectoryTime);
   }

   public static PelvisPosePacket createGoToHomePelvisPosePacket(double trajectoryTime)
   {
      return new PelvisPosePacket(null, null, true, trajectoryTime);
   }

   public static ChestOrientationPacket createChestOrientationPacket(Quat4d orientation, double trajectoryTime)
   {
      return new ChestOrientationPacket(orientation, false, trajectoryTime);
   }

   public static ChestOrientationPacket createGoToHomeChestOrientationPacket(double trajectoryTime)
   {
      return new ChestOrientationPacket(null, true, trajectoryTime);
   }

}
