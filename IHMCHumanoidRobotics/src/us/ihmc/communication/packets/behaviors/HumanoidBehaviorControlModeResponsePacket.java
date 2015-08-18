package us.ihmc.communication.packets.behaviors;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.tools.random.RandomTools;

public class HumanoidBehaviorControlModeResponsePacket extends Packet<HumanoidBehaviorControlModeResponsePacket>
{
   public HumanoidBehaviorControlModeEnum requestedControl;

   // empty constructor for deserialization
   public HumanoidBehaviorControlModeResponsePacket()
   {
   }

   public HumanoidBehaviorControlModeResponsePacket(HumanoidBehaviorControlModeEnum requestedControl)
   {
      this.requestedControl = requestedControl;
   }

   public HumanoidBehaviorControlModeEnum getRequestedControl()
   {
      return requestedControl;
   }

   @Override
   public boolean epsilonEquals(HumanoidBehaviorControlModeResponsePacket other, double epsilon)
   {
      return this.requestedControl.equals(other.requestedControl);
   }

   public HumanoidBehaviorControlModeResponsePacket(Random random)
   {
      this(HumanoidBehaviorControlModeEnum.values[RandomTools.generateRandomInt(random, 0, HumanoidBehaviorControlModeEnum.values.length - 1)]);

   }
}
