package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class BehaviorControlModeResponsePacket extends Packet<BehaviorControlModeResponsePacket>
{
   public static final byte STOP = 0;
   public static final byte PAUSE = 1;
   public static final byte RESUME = 2;

   public byte behaviorControlModeEnumRequest;

   // empty constructor for deserialization
   public BehaviorControlModeResponsePacket()
   {
   }

   @Override
   public void set(BehaviorControlModeResponsePacket other)
   {
      setPacketInformation(other);
      behaviorControlModeEnumRequest = other.behaviorControlModeEnumRequest;
   }

   public byte getBehaviorControlModeEnumRequest()
   {
      return behaviorControlModeEnumRequest;
   }

   @Override
   public boolean epsilonEquals(BehaviorControlModeResponsePacket other, double epsilon)
   {
      return this.behaviorControlModeEnumRequest == other.behaviorControlModeEnumRequest;
   }
}
