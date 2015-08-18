package us.ihmc.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class ControlStatusPacket extends Packet<ControlStatusPacket>
{
   public enum ControlStatus
   {
      HAND_TRAJECTORY_INFEASIBLE("hand trajectory failed to converge, freezing joints");


//      public final String message;

      ControlStatus(String message)
      {
//         this.message = message;
      }

//      @Override
//      public String toString()
//      {
//         return message;
//      }
   }

   public RobotSide robotSide;
   public ControlStatus controlStatus;

   public ControlStatusPacket()
   {
   }

   public ControlStatusPacket(RobotSide robotSide, ControlStatus controlStatus)
   {
      this.robotSide = robotSide;
      this.controlStatus = controlStatus;
   }

   @Override
   public String toString()
   {
      return robotSide.getCamelCaseNameForStartOfExpression() + " " + controlStatus.toString();
   }

   @Override
   public boolean equals(Object otherObject)
   {
      if (otherObject instanceof ControlStatusPacket)
      {
         return epsilonEquals((ControlStatusPacket) otherObject, 0);
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(ControlStatusPacket other, double epsilon)
   {
      return other.robotSide == robotSide;
   }

   public ControlStatusPacket(Random random)
   {
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT, ControlStatus.values()[random.nextInt(ControlStatus.values().length)]);
   }
}
