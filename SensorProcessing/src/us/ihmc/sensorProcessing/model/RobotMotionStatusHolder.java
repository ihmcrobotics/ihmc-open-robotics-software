package us.ihmc.sensorProcessing.model;

import java.util.concurrent.atomic.AtomicReference;

public class RobotMotionStatusHolder
{
   private final AtomicReference<RobotMotionStatus> currentRobotMotionStatus = new AtomicReference<RobotMotionStatus>(RobotMotionStatus.STANDING);

   public RobotMotionStatusHolder()
   {
   }

   public RobotMotionStatus getCurrentRobotMotionStatus()
   {
      return currentRobotMotionStatus.get();
   }

   public void setCurrentRobotMotionStatus(RobotMotionStatus currentRobotMotionStatus)
   {
      this.currentRobotMotionStatus.set(currentRobotMotionStatus);
   }
}
