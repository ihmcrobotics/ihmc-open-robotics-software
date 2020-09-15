package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.Timer;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.walkingController.ControllerStatusTracker;

public class LookAndStepReset
{
   private SingleThreadSizeOneQueueExecutor executor;
   private StatusLogger statusLogger;
   private ControllerStatusTracker controllerStatusTracker;
   private LookAndStepBehaviorParametersReadOnly lookAndStepParameters;
   private Runnable runBeforeWaitWalking;
   private Runnable runAfterWaitWalking;
   private Timer resetTimer = new Timer();

   public void initialize(StatusLogger statusLogger,
                          ControllerStatusTracker controllerStatusTracker,
                          LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                          Runnable runBeforeWaitWalking,
                          Runnable runAfterWaitWalking)
   {
      this.statusLogger = statusLogger;
      this.controllerStatusTracker = controllerStatusTracker;
      this.lookAndStepParameters = lookAndStepBehaviorParameters;
      this.runBeforeWaitWalking = runBeforeWaitWalking;
      this.runAfterWaitWalking = runAfterWaitWalking;

      executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());
   }

   public void queueReset()
   {
      resetTimer.reset();
      executor.queueExecution(this::performReset);
   }

   private void performReset()
   {
      statusLogger.error("Resetting behavior");
      runBeforeWaitWalking.run();
//      if (controllerStatusTracker.isWalking())
//      {
//         statusLogger.info("Waiting for walking to finish");
//         controllerStatusTracker.getFinishedWalkingNotification().blockingPoll();
//      statusLogger.info("Finished walking. Waiting for remaining {} s", lookAndStepParameters.getResetDuration());
//      }
      statusLogger.info("Waiting for {} s to expire", lookAndStepParameters.getResetDuration());
      resetTimer.sleepUntilExpiration(lookAndStepParameters.getResetDuration());
      runAfterWaitWalking.run();
      statusLogger.info("Reset complete");
   }
}
