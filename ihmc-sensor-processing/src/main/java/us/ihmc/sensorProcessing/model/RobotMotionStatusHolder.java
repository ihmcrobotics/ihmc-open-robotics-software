package us.ihmc.sensorProcessing.model;

import java.util.concurrent.atomic.AtomicReference;

public class RobotMotionStatusHolder
{
   private final AtomicReference<RobotMotionStatus> currentRobotMotionStatus = new AtomicReference<RobotMotionStatus>(RobotMotionStatus.UNKNOWN);

   public RobotMotionStatus getCurrentRobotMotionStatus()
   {
      return currentRobotMotionStatus.get();
   }

   public void setCurrentRobotMotionStatus(RobotMotionStatus currentRobotMotionStatus)
   {
      this.currentRobotMotionStatus.set(currentRobotMotionStatus);
   }

   public void set(RobotMotionStatusHolder other)
   {
      setCurrentRobotMotionStatus(other.getCurrentRobotMotionStatus());
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof RobotMotionStatusHolder)
      {
         RobotMotionStatusHolder other = (RobotMotionStatusHolder) obj;
         if (currentRobotMotionStatus.get() != other.currentRobotMotionStatus.get())
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
