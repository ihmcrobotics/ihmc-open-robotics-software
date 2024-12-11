package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.log.BehaviorTreeNodeMessageLogger;
import us.ihmc.tools.NonWallTimer;

/**
 * Tracks the elapsed time and time limit for an executing trajectory.
 */
public class TrajectoryDurationTracker
{
   private boolean timeIsUp;
   private boolean hitTimeLimit;
   private final NonWallTimer executionTimer = new NonWallTimer();

   private double nominalExecutionDuration;
   private double timeout;

   public void update(double robotTime)
   {
      executionTimer.update(robotTime);
   }

   public void reset()
   {
      executionTimer.reset();
      timeIsUp = false;
      hitTimeLimit = false;
   }

   public void computeExecutionTimings(double nominalExecutionDuration)
   {
      computeExecutionTimings(nominalExecutionDuration, nominalExecutionDuration * 1.5);
   }

   public void computeExecutionTimings(double nominalExecutionDuration, double timeout)
   {
      this.nominalExecutionDuration = nominalExecutionDuration;
      this.timeout = timeout;

      timeIsUp = !executionTimer.isRunning(nominalExecutionDuration);
      // Default timeout is 50% longer than nominal TODO: Introduce parameter
      hitTimeLimit = !executionTimer.isRunning(timeout);
   }

   public boolean getTimeIsUp()
   {
      return timeIsUp;
   }

   public boolean getHitTimeLimit(BehaviorTreeNodeMessageLogger logger)
   {
      if (hitTimeLimit)
      {
         logger.error(1, "Task execution timed out. %.2f > %.2f Nominal: %.2f".formatted(executionTimer.getElapsedTime(), timeout, nominalExecutionDuration));
      }

      return hitTimeLimit;
   }

   public double getElapsedTime()
   {
      return executionTimer.getElapsedTime();
   }

   public double getNominalExecutionDuration()
   {
      return nominalExecutionDuration;
   }

   public double getTimeout()
   {
      return timeout;
   }
}
