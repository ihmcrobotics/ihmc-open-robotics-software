package us.ihmc.humanoidRobotics.communication.packets.bdi;

import us.ihmc.communication.packets.Packet;

public class BDIBehaviorCommandPacket extends Packet<BDIBehaviorCommandPacket>
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
