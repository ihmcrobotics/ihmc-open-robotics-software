package us.ihmc.behaviors.lookAndStep;

import ihmc_common_msgs.msg.dds.PoseListMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.property.ROS2StoredPropertySet;
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

      ROS2StoredPropertySet<LookAndStepBehaviorParametersBasics> ros2LookAndStepParameters = lookAndStep.ros2LookAndStepParameters;
      ros2LookAndStepParameters.update();
      lookAndStep.statusLogger.info("Waiting for {} s to expire", ros2LookAndStepParameters.getStoredPropertySet().getResetDuration());
      resetTimer.sleepUntilExpiration(ros2LookAndStepParameters.getStoredPropertySet().getResetDuration());

      runAfterWaitingForWalkingToFinish();

      lookAndStep.statusLogger.info("Reset complete");
   }

   private void runBeforeWaitingForWalkingToFinish()
   {
      lookAndStep.isBeingReset.set(true);
      lookAndStep.behaviorStateReference.set(LookAndStepBehavior.State.RESET);

      lookAndStep.operatorReviewEnabledInput.set(true);
      lookAndStep.helper.publish(OPERATOR_REVIEW_ENABLED_STATUS, true);

      lookAndStep.bodyPathPlanning.reset();
      lookAndStep.bodyPathLocalization.reset();
      lookAndStep.footstepPlanning.reset();
      lookAndStep.stepping.reset();

      lookAndStep.robotInterface.pauseWalking();
   }

   private void runAfterWaitingForWalkingToFinish()
   {
      lookAndStep.bodyPathPlanning.acceptGoal(null);
      lookAndStep.helper.publish(RESET_FOR_UI);
      lookAndStep.imminentStanceTracker.clear();
      lookAndStep.controllerStatusTracker.reset();

      // REAStateRequestMessage clearMessage = new REAStateRequestMessage();
      // clearMessage.setRequestClear(true);
      // statusLogger.info("Requesting clear REA");
      // helper.publish(PerceptionAPI.REA_STATE_REQUEST, clearMessage);

      lookAndStep.helper.publish(PLANAR_REGIONS_FOR_UI, new PlanarRegionsListMessage());
      lookAndStep.helper.publish(BODY_PATH_PLAN_FOR_UI, new PoseListMessage());

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
