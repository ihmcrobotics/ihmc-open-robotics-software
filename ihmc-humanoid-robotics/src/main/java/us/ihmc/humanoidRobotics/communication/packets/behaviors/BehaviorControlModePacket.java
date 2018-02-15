package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;

public class BehaviorControlModePacket extends Packet<BehaviorControlModePacket>
{
   public static enum BehaviorControlModeEnum
   {
      STOP, PAUSE, RESUME;

      public static final BehaviorControlModeEnum[] values = values();
   }

   public BehaviorControlModeEnum requestedControl;

   // empty constructor for deserialization
   public BehaviorControlModePacket()
   {
   }

   @Override
   public void set(BehaviorControlModePacket other)
   {
      requestedControl = other.requestedControl;
   }

   public BehaviorControlModeEnum getRequestedControl()
   {
      return requestedControl;
   }

   public boolean epsilonEquals(BehaviorControlModePacket other, double epsilon)
   {
      return this.requestedControl.equals(other.requestedControl);
   }
}
