package us.ihmc.humanoidRobotics.communication.packets.bdi;

import us.ihmc.communication.packets.Packet;

public class BDIBehaviorCommandPacket extends Packet<BDIBehaviorCommandPacket>
{
   public byte atlasBDIRobotBehavior;
   public boolean stop = false;

   public BDIBehaviorCommandPacket()
   {
   }

   @Override
   public void set(BDIBehaviorCommandPacket other)
   {
      setPacketInformation(other);
      atlasBDIRobotBehavior = other.atlasBDIRobotBehavior;
      stop = other.stop;
   }

   @Override
   public boolean equals(Object other)
   {
      return (other instanceof BDIBehaviorCommandPacket) && epsilonEquals((BDIBehaviorCommandPacket) other, 0);
   }

   @Override
   public boolean epsilonEquals(BDIBehaviorCommandPacket other, double epsilon)
   {
      return (other.atlasBDIRobotBehavior == atlasBDIRobotBehavior) && (other.stop == stop);
   }
}
