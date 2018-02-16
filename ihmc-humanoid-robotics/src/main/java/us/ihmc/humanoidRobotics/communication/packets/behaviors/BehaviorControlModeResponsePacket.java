package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class BehaviorControlModeResponsePacket extends Packet<BehaviorControlModeResponsePacket>
{
   public byte requestedControl;

   // empty constructor for deserialization
   public BehaviorControlModeResponsePacket()
   {
   }

   @Override
   public void set(BehaviorControlModeResponsePacket other)
   {
      setPacketInformation(other);
      requestedControl = other.requestedControl;
   }

   public byte getRequestedControl()
   {
      return requestedControl;
   }

   @Override
   public boolean epsilonEquals(BehaviorControlModeResponsePacket other, double epsilon)
   {
      return this.requestedControl == other.requestedControl;
   }
}
