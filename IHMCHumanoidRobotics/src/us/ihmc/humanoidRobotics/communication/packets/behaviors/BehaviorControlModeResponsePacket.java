package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.robotics.random.RandomTools;

public class BehaviorControlModeResponsePacket extends Packet<BehaviorControlModeResponsePacket>
{
   public BehaviorControlModeEnum requestedControl;

   // empty constructor for deserialization
   public BehaviorControlModeResponsePacket()
   {
   }

   public BehaviorControlModeResponsePacket(BehaviorControlModeEnum requestedControl)
   {
      this.requestedControl = requestedControl;
   }

   public BehaviorControlModeEnum getRequestedControl()
   {
      return requestedControl;
   }

   @Override
   public boolean epsilonEquals(BehaviorControlModeResponsePacket other, double epsilon)
   {
      return this.requestedControl.equals(other.requestedControl);
   }

   public BehaviorControlModeResponsePacket(Random random)
   {
      this(BehaviorControlModeEnum.values[RandomTools.generateRandomInt(random, 0, BehaviorControlModeEnum.values.length - 1)]);

   }
}
