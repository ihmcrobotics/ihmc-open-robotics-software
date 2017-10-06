package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
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

   public BehaviorControlModePacket(BehaviorControlModeEnum requestedControl)
   {
      this.requestedControl = requestedControl;
   }

   public BehaviorControlModeEnum getRequestedControl()
   {
      return requestedControl;
   }

   public boolean epsilonEquals(BehaviorControlModePacket other, double epsilon)
   {
      return this.requestedControl.equals(other.requestedControl);
   }

   public BehaviorControlModePacket(Random random)
   {
      this(BehaviorControlModeEnum.values[RandomNumbers.nextInt(random, 0, BehaviorControlModeEnum.values.length - 1)]);

   }
}
