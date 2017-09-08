package us.ihmc.humanoidRobotics.communication.packets.driving;

import java.util.Random;

import us.ihmc.communication.packets.Packet;


public class DrivingTrajectoryPacket extends Packet<DrivingTrajectoryPacket>
{
   public enum Length {SHORT, MEDIUM, LONG}

   public Length length;
   public int trajectory;

   public DrivingTrajectoryPacket()
   {
   }
   
   public DrivingTrajectoryPacket(Length length, int trajectory)
   {
      this.length = length;
      this.trajectory = trajectory;
   }

   public Length getLength()
   {
      return length;
   }

   public int getTrajectory()
   {
      return trajectory;
   }

   public boolean equals(Object other)
   {
      if (other instanceof DrivingTrajectoryPacket)
      {
         return epsilonEquals((DrivingTrajectoryPacket) other, 0);
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(DrivingTrajectoryPacket other, double epsilon)
   {
      return (other.getLength() == getLength()) && (other.getTrajectory() == getTrajectory());
   }

   public DrivingTrajectoryPacket(Random random)
   {
      this(Length.values()[random.nextInt(Length.values().length)], random.nextInt(63));
   }
}
