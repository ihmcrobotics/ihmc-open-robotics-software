package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class BehaviorControlModePacket extends Packet<BehaviorControlModePacket>
{
   public byte requestedControl;

   // empty constructor for deserialization
   public BehaviorControlModePacket()
   {
   }

   @Override
   public void set(BehaviorControlModePacket other)
   {
      requestedControl = other.requestedControl;
   }

   public byte getRequestedControl()
   {
      return requestedControl;
   }

   public boolean epsilonEquals(BehaviorControlModePacket other, double epsilon)
   {
      return this.requestedControl == other.requestedControl;
   }
}
