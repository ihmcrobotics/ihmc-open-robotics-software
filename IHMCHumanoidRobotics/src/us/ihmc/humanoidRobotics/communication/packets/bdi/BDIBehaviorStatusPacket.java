package us.ihmc.humanoidRobotics.communication.packets.bdi;

import java.util.Random;

import us.ihmc.communication.packets.Packet;

public class BDIBehaviorStatusPacket extends Packet<BDIBehaviorStatusPacket>
{
   public BDIRobotBehavior currentBehavior;

   public BDIBehaviorStatusPacket()
   {
   }

   public BDIBehaviorStatusPacket(BDIRobotBehavior currentBehavior)
   {
      this.currentBehavior = currentBehavior;
   }

   @Override
   public boolean equals(Object other)
   {
      return epsilonEquals((BDIBehaviorStatusPacket) other, 0);
   }

   @Override
   public boolean epsilonEquals(BDIBehaviorStatusPacket other, double epsilon)
   {
      return (other instanceof BDIBehaviorStatusPacket) && (other).currentBehavior == currentBehavior;
   }

   public BDIBehaviorStatusPacket(Random random)
   {
      this(BDIRobotBehavior.values[random.nextInt(BDIRobotBehavior.values.length)]);
   }
}
