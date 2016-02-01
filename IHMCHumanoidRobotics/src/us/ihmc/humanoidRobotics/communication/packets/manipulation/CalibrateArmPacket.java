package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;

public class CalibrateArmPacket extends Packet<CalibrateArmPacket>
{
   @Override
   public boolean equals(Object other)
   {
      return other instanceof CalibrateArmPacket;
   }

   @Override
   public boolean epsilonEquals(CalibrateArmPacket other, double epsilon)
   {
      return true;
   }

   public CalibrateArmPacket()
   {
   }

   public CalibrateArmPacket(Random random)
   {
   }
}
