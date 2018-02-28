package us.ihmc.humanoidRobotics.communication.packets.bdi;

import us.ihmc.communication.packets.Packet;

public class BDIBehaviorStatusPacket extends Packet<BDIBehaviorStatusPacket>
{
   public static final byte NONE = 0;
   public static final byte FREEZE = 1;
   public static final byte STAND_PREP = 2;
   public static final byte STAND = 3;
   public static final byte WALK = 4;
   public static final byte STEP = 5;
   public static final byte MANIPULATE = 6;
   public static final byte USER = 7;
   public static final byte CALIBRATE = 8;
   public static final byte SOFT_STOP = 9;

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
