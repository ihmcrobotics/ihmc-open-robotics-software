package us.ihmc.humanoidRobotics.communication.packets.bdi;

import us.ihmc.communication.packets.Packet;

public class BDIBehaviorCommandPacket extends Packet<BDIBehaviorCommandPacket>
{
   public BDIRobotBehavior atlasRobotBehavior;
   public boolean stop = false;

   public BDIBehaviorCommandPacket()
   {
   }

   public BDIBehaviorCommandPacket(BDIRobotBehavior atlasRobotBehavior)
   {
      this.atlasRobotBehavior = atlasRobotBehavior;
   }

   public BDIBehaviorCommandPacket(boolean stop)
   {
      this.stop = stop;
   }

   @Override
   public boolean equals(Object other)
   {
      return (other instanceof BDIBehaviorCommandPacket) && epsilonEquals((BDIBehaviorCommandPacket) other, 0);
   }

   @Override
   public boolean epsilonEquals(BDIBehaviorCommandPacket other, double epsilon)
   {
      return (other.atlasRobotBehavior == atlasRobotBehavior) && (other.stop == stop);
   }
}
