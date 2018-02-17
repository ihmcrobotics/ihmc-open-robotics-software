package us.ihmc.humanoidRobotics.communication.packets.bdi;

import us.ihmc.communication.packets.Packet;

public class BDIBehaviorStatusPacket extends Packet<BDIBehaviorStatusPacket>
{
   public byte currentBDIRobotBehavior;

   public BDIBehaviorStatusPacket()
   {
   }

   @Override
   public void set(BDIBehaviorStatusPacket other)
   {
      setPacketInformation(other);
      currentBDIRobotBehavior = other.currentBDIRobotBehavior;
   }

   @Override
   public boolean equals(Object other)
   {
      return epsilonEquals((BDIBehaviorStatusPacket) other, 0);
   }

   @Override
   public boolean epsilonEquals(BDIBehaviorStatusPacket other, double epsilon)
   {
      return other.currentBDIRobotBehavior == currentBDIRobotBehavior;
   }
}
