package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.random.RandomTools;

public class BehaviorControlModePacket extends Packet<BehaviorControlModePacket>
{
   public static enum BehaviorControlModeEnum
   {
      ENABLE_ACTIONS, STOP, PAUSE, RESUME;

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
      this(BehaviorControlModeEnum.values[RandomTools.generateRandomInt(random, 0, BehaviorControlModeEnum.values.length - 1)]);

   }
}
