package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class BehaviorControlModePacket extends Packet<BehaviorControlModePacket>
{
   public static final byte STOP = 0;
   public static final byte PAUSE = 1;
   public static final byte RESUME = 2;

   public byte behaviorControlModeEnumRequest;

   // empty constructor for deserialization
   public BehaviorControlModePacket()
   {
   }

   @Override
   public void set(BehaviorControlModePacket other)
   {
      behaviorControlModeEnumRequest = other.behaviorControlModeEnumRequest;
   }

   public byte getRequestedControl()
   {
      return behaviorControlModeEnumRequest;
   }

   public boolean epsilonEquals(BehaviorControlModePacket other, double epsilon)
   {
      return this.behaviorControlModeEnumRequest == other.behaviorControlModeEnumRequest;
   }
}
