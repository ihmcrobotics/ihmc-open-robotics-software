package us.ihmc.humanoidRobotics.communication.packets.driving;

import java.util.Random;

import us.ihmc.communication.packets.Packet;

public class DrivingStatePacket extends Packet<DrivingStatePacket>
{
   public enum DrivingState
   {
      // Setting
      START_MANUAL, START_AUTONOMOUS, START_AUTONOMOUS2, ABORT, COMPUTE_GROUNDPLANE, DISABLE,

      // Status
      MANUAL, SEMI_AUTONOMOUS, AUTONOMOUS, AUTONOMOUS2, STARTED_TRAJECTORY, FINISHED_TRAJECTORY, ABORTED_TRAJECTORY, STOPPED_AUTONOMOUS_MODE, DISABLED,
   }

   public DrivingState drivingState;

   public DrivingStatePacket()
   {
   }
   
   public DrivingStatePacket(DrivingState drivingState)
   {
      this.drivingState = drivingState;
   }

   public DrivingState getDrivingState()
   {
      return drivingState;
   }

   public boolean equals(Object other)
   {
      if (other instanceof DrivingStatePacket)
      {
         return epsilonEquals((DrivingStatePacket) other, 0);
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(DrivingStatePacket other, double epsilon)
   {
      return other.getDrivingState() == getDrivingState();
   }

   public DrivingStatePacket(Random random)
   {
      this(DrivingState.values()[random.nextInt(DrivingState.values().length)]);

   }
}
