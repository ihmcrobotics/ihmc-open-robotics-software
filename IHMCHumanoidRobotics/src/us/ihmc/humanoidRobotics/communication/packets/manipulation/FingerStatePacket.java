package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.FingerState;
import us.ihmc.robotics.robotSide.RobotSide;

public class FingerStatePacket extends Packet<FingerStatePacket>
{
   public RobotSide robotSide;
   public FingerState fingerState;

   public FingerStatePacket()
   {
      // Empty constructor for deserialization
   }

   public FingerStatePacket(RobotSide robotSide, FingerState fingerState)
   {
      this.robotSide = robotSide;
      this.fingerState = fingerState;
   }

   public FingerState getFingerState()
   {
      return fingerState;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }


   public boolean equals(Object obj)
   {
      return ((obj instanceof FingerStatePacket) && this.epsilonEquals((FingerStatePacket) obj, 0));
   }

   public String toString()
   {
      return robotSide.toString() + " State= " + fingerState.toString();
   }

   @Override
   public boolean epsilonEquals(FingerStatePacket other, double epsilon)
   {
      boolean ret = (this.getRobotSide() == other.getRobotSide());
      ret &= (this.getFingerState().equals(other.getFingerState()));

      return ret;
   }

   public FingerStatePacket(Random random)
   {
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT, FingerState.values()[random.nextInt(FingerState.values().length)]);

   }
}
