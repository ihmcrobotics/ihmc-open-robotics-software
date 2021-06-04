package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepReset
{
   private LookAndStepBehavior lookAndStep;
   private ResettableExceptionHandlingExecutorService executor;
   private final Timer resetTimer = new Timer();

   public void initialize(LookAndStepBehavior lookAndStep)
   {
      this.lookAndStep = lookAndStep;

      executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   }

   public void queueReset()
   {
      resetTimer.reset();
      executor.clearQueueAndExecute(this::performReset);
   }

   private void performReset()
   {
      lookAndStep.statusLogger.info("Resetting behavior");

      runBeforeWaitingForWalkingToFinish();

//      if (controllerStatusTracker.isWalking())
//      {
//         statusLogger.info("Waiting for walking to finish");
//         controllerStatusTracker.getFinishedWalkingNotification().blockingPoll();
//      statusLogger.info("Finished walking. Waiting for remaining {} s", lookAndStepParameters.getResetDuration());
//      }

      lookAndStep.statusLogger.info("Waiting for {} s to expire", lookAndStep.lookAndStepParameters.getResetDuration());
      resetTimer.sleepUntilExpiration(lookAndStep.lookAndStepParameters.getResetDuration());

      runAfterWaitingForWalkingToFinish();

      lookAndStep.statusLogger.info("Reset complete");
   }

   private void runBeforeWaitingForWalkingToFinish()
   {
      lookAndStep.isBeingReset.set(true);
      lookAndStep.behaviorStateReference.set(LookAndStepBehavior.State.RESET);

      lookAndStep.operatorReviewEnabledInput.set(true);
      lookAndStep.helper.publish(OperatorReviewEnabledToUI, true);

      lookAndStep.bodyPathPlanning.reset();
      lookAndStep.bodyPathLocalization.reset();
      lookAndStep.footstepPlanning.reset();
      lookAndStep.stepping.reset();

      lookAndStep.robotInterface.pauseWalking();
   }

   private void runAfterWaitingForWalkingToFinish()
   {
      lookAndStep.bodyPathPlanning.acceptGoal(null);
      lookAndStep.lastStanceSide.set(null);
      lookAndStep.helper.publish(ResetForUI);
      lookAndStep.lastCommandedFootsteps.clear();
      lookAndStep.controllerStatusTracker.reset();

      // REAStateRequestMessage clearMessage = new REAStateRequestMessage();
      // clearMessage.setRequestClear(true);
      // statusLogger.info("Requesting clear REA");
      // helper.publish(ROS2Tools.REA_STATE_REQUEST, clearMessage);

      lookAndStep.helper.publish(PlanarRegionsForUI, new PlanarRegionsList());
      lookAndStep.helper.publish(BodyPathPlanForUI, new ArrayList<>());

      lookAndStep.statusLogger.info("Clearing SLAM");
      lookAndStep.helper.publish(SLAMModuleAPI.CLEAR);

      lookAndStep.isBeingReset.set(false);
      lookAndStep.behaviorStateReference.set(LookAndStepBehavior.State.BODY_PATH_PLANNING);
   }

   public void destroy()
   {
      executor.destroy();
   }
}
