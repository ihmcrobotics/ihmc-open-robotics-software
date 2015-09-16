package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.random.RandomTools;

public class HumanoidBehaviorControlModePacket extends Packet<HumanoidBehaviorControlModePacket>
{
   public static enum HumanoidBehaviorControlModeEnum
   {
      ENABLE_ACTIONS, STOP, PAUSE, RESUME;

      public static final HumanoidBehaviorControlModeEnum[] values = values();
   }

   public HumanoidBehaviorControlModeEnum requestedControl;

   // empty constructor for deserialization
   public HumanoidBehaviorControlModePacket()
   {
   }

   public HumanoidBehaviorControlModePacket(HumanoidBehaviorControlModeEnum requestedControl)
   {
      this.requestedControl = requestedControl;
   }

   public HumanoidBehaviorControlModeEnum getRequestedControl()
   {
      return requestedControl;
   }

   public boolean epsilonEquals(HumanoidBehaviorControlModePacket other, double epsilon)
   {
      return this.requestedControl.equals(other.requestedControl);
   }

   public HumanoidBehaviorControlModePacket(Random random)
   {
      this(HumanoidBehaviorControlModeEnum.values[RandomTools.generateRandomInt(random, 0, HumanoidBehaviorControlModeEnum.values.length - 1)]);

   }
}
