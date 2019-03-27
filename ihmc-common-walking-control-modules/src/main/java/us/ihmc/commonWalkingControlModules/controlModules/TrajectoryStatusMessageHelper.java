package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;

/**
 * 
 * @author Sylvain Bertrand
 */
public abstract class TrajectoryStatusMessageHelper<M>
{
   public static final long NULL_SEQUENCE_ID = -1L;
   private final RecyclingArrayList<TrajectoryTimeInfo> inputs = new RecyclingArrayList<>(TrajectoryTimeInfo.class);

   private final TrajectoryStatus currentTrajectoryStatus = new TrajectoryStatus();

   public TrajectoryStatusMessageHelper()
   {
   }

   public void clear()
   {
      inputs.clear();
      currentTrajectoryStatus.sequenceID = -1L;
      currentTrajectoryStatus.timeInTrajectory = 0.0;
      currentTrajectoryStatus.status = null;
   }

   public void registerNewTrajectory(long trajectorySequenceID, double trajectoryStartTime, double trajectoryEndTime)
   {
      inputs.add().set(trajectorySequenceID, trajectoryStartTime, trajectoryEndTime);
   }

   public void updateWithTimeInTrajectory(double timeInTrajectory)
   {
      if (inputs.isEmpty())
      {
         currentTrajectoryStatus.sequenceID = -1L;
         currentTrajectoryStatus.status = null;
         return;
      }

      TrajectoryTimeInfo currentTrajectory = inputs.get(0);
      currentTrajectoryStatus.sequenceID = currentTrajectory.sequenceID;
      currentTrajectoryStatus.timeInTrajectory = timeInTrajectory;

      if (!currentTrajectory.hasAlreadyStarted && timeInTrajectory >= currentTrajectory.startTime)
      {
         currentTrajectoryStatus.status = TrajectoryExecutionStatus.STARTED;
         currentTrajectory.hasAlreadyStarted = true;
      }
      else if (timeInTrajectory >= currentTrajectory.endTime)
      {
         currentTrajectoryStatus.status = TrajectoryExecutionStatus.COMPLETED;
      }
      else
      {
         currentTrajectoryStatus.status = null;
      }
   }

   protected TrajectoryStatus pollStatus()
   {
      if (!inputs.isEmpty() && currentTrajectoryStatus.status != null)
      {
         if (currentTrajectoryStatus.status == TrajectoryExecutionStatus.COMPLETED)
            inputs.remove(0);

         return currentTrajectoryStatus;
      }
      else
      {
         return null;
      }
   }

   public static class TrajectoryTimeInfo
   {
      private long sequenceID;
      private double startTime;
      private double endTime;
      private boolean hasAlreadyStarted;

      public TrajectoryTimeInfo()
      {
      }

      public void set(long sequenceID, double startTime, double endTime)
      {
         this.sequenceID = sequenceID;
         this.startTime = startTime;
         this.endTime = endTime;
         hasAlreadyStarted = false;
      }
   }

   protected static class TrajectoryStatus
   {
      private long sequenceID = -1L;
      private double timeInTrajectory = 0.0;
      private TrajectoryExecutionStatus status = null;

      public long getSequenceID()
      {
         return sequenceID;
      }

      public double getTimeInTrajectory()
      {
         return timeInTrajectory;
      }

      public TrajectoryExecutionStatus getStatus()
      {
         return status;
      }
   }
}
