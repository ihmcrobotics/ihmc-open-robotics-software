package us.ihmc.gr00t;

import controller_msgs.ArmTrajectoryMessage;
import controller_msgs.NeckTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.variable.YoDouble;

import java.nio.DoubleBuffer;
import java.util.List;
import java.util.function.Consumer;

/** Owns accepted policy chunks, IK plans, controller publication, convergence, and RDX preview state. */
final class Gr00tHumanoidPlanExecutor
{
   enum Completion
   {
      NONE, CONVERGED, TIMED_OUT
   }

   private final ROS2SyncedRobotModel syncedRobot;
   private final Gr00tHumanoidConfiguration configuration;
   private final Gr00tActionDecoder<Gr00tHumanoidAction> actionDecoder;
   private final Gr00tHumanoidDiagnostics diagnostics;
   private final Consumer<String> statusUpdater;
   private final ROS2Publisher<NeckTrajectoryMessage> neckTrajectoryPublisher;
   private final ROS2Publisher<ArmTrajectoryMessage> armTrajectoryPublisher;
   private final Gr00tHumanoidArmPlanner armPlanner;
   private final Gr00tHandController hands;

   private volatile Gr00tHumanoidTask.PreviewState pendingPreview;
   private volatile String planStatus = "Waiting for a policy action chunk";
   private volatile double planDuration = Double.NaN;
   private volatile double worstIKQuality = Double.NaN;
   private volatile boolean allowPoorIKForTesting;
   private volatile boolean executeAcceptedGoalRequested;
   private volatile boolean trajectoryEverPublished;
   private volatile boolean trajectoryPublished;
   private volatile Gr00tHumanoidAction acceptedGoal;
   private volatile List<Gr00tHumanoidAction> acceptedActionChunk;
   private volatile boolean controlRobot;
   private volatile boolean controlNeck;
   private Gr00tHumanoidArmPlanner.Plan activePlan;
   private long activePlanStartNanos = -1L;
   private int lastPublishedHandSample;
   private long previewSequence;

   Gr00tHumanoidPlanExecutor(AsyncROS2Node ros2Node,
                             DRCRobotModel robotModel,
                             ROS2SyncedRobotModel syncedRobot,
                             Gr00tHumanoidConfiguration configuration,
                             Gr00tActionDecoder<Gr00tHumanoidAction> actionDecoder,
                             Gr00tHumanoidDiagnostics diagnostics,
                             Gr00tHandController hands,
                             Consumer<String> statusUpdater)
   {
      this.syncedRobot = syncedRobot;
      this.configuration = configuration;
      this.actionDecoder = actionDecoder;
      this.diagnostics = diagnostics;
      this.hands = hands;
      this.statusUpdater = statusUpdater;
      neckTrajectoryPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(NeckTrajectoryMessage.class, robotModel.getSimpleRobotName()));
      armTrajectoryPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(ArmTrajectoryMessage.class, robotModel.getSimpleRobotName()));
      armPlanner = new Gr00tHumanoidArmPlanner(robotModel, syncedRobot, configuration);
      diagnostics.actionWristPoses.forEach(pose -> pose.setToNaN());
   }

   void updateBeforeExecution(boolean running)
   {
      if (executeAcceptedGoalRequested && running && controlRobot && acceptedActionChunk != null)
      {
         executeAcceptedGoalRequested = false;
         rebuildAcceptedPlan();
      }
      updatePreview();
   }

   void updatePreview()
   {
      if (activePlan == null || activePlanStartNanos < 0L)
         return;
      double elapsed = Math.max(0.0, (System.nanoTime() - activePlanStartNanos) * 1.0e-9);
      int upperSample = 1;
      while (upperSample < activePlan.trajectoryTimes.length - 1 && activePlan.trajectoryTimes[upperSample] < elapsed)
         upperSample++;
      int lowerSample = Math.max(0, upperSample - 1);
      double segmentDuration = activePlan.trajectoryTimes[upperSample] - activePlan.trajectoryTimes[lowerSample];
      double alpha = segmentDuration > 0.0
                     ? MathTools.clamp((elapsed - activePlan.trajectoryTimes[lowerSample]) / segmentDuration, 0.0, 1.0)
                     : 1.0;
      double[] previewJoints = new double[activePlan.fullJointPositions[lowerSample].length];
      for (int joint = 0; joint < previewJoints.length; joint++)
      {
         previewJoints[joint] = (1.0 - alpha) * activePlan.fullJointPositions[lowerSample][joint]
                                + alpha * activePlan.fullJointPositions[upperSample][joint];
      }
      pendingPreview = new Gr00tHumanoidTask.PreviewState(activePlan.rootPose, previewJoints, ++previewSequence);
      publishDueHandTarget(elapsed);
   }

   void processActionChunk(DoubleBuffer actionChunk, int realActionCount)
   {
      List<Gr00tHumanoidAction> candidates = actionDecoder.decode(actionChunk, realActionCount);
      if (candidates.isEmpty())
      {
         updatePlanStatus("Action chunk rejected: no valid finite action rows");
         return;
      }
      Gr00tHumanoidAction candidate = candidates.get(candidates.size() - 1);

      diagnostics.actionWristPoses.get(RobotSide.LEFT).set(candidate.getWristPoseReadOnly(RobotSide.LEFT));
      diagnostics.actionWristPoses.get(RobotSide.RIGHT).set(candidate.getWristPoseReadOnly(RobotSide.RIGHT));
      for (RobotSide side : RobotSide.values)
      {
         YoDouble[] actionFingerYos = diagnostics.actionFingers.get(side);
         double[] handTargets = candidate.getHandTargets(side);
         for (int finger = 0; finger < Math.min(actionFingerYos.length, handTargets.length); finger++)
            actionFingerYos[finger].set(handTargets[finger]);
      }

      Gr00tHumanoidArmPlanner.Plan plan = buildPlan(candidates, !trajectoryEverPublished);
      if (plan == null)
         return;

      acceptedGoal = candidate;
      acceptedActionChunk = candidates;
      installPlan(plan);
      diagnostics.numberOfActionsTaken.add(candidates.size());
      if (controlRobot)
      {
         executeAcceptedGoalRequested = false;
         publishPlan(plan);
      }
   }

   Completion pollCompletion()
   {
      if (activePlan == null || acceptedGoal == null || activePlanStartNanos < 0L)
         return Completion.NONE;

      double elapsedSeconds = (System.nanoTime() - activePlanStartNanos) * 1.0e-9;
      if (!controlRobot)
         return Gr00tHumanoidTrajectoryTools.stageHoldRemainingSeconds(configuration, elapsedSeconds, activePlan.duration) <= 0.0
                ? Completion.CONVERGED : Completion.NONE;
      if (!trajectoryPublished)
         return Completion.NONE;
      boolean endpointConverged = isCurrentWristNear();
      if (!Gr00tHumanoidTrajectoryTools.isActionChunkComplete(configuration, elapsedSeconds, activePlan.duration, endpointConverged))
      {
         if (Gr00tHumanoidTrajectoryTools.stageHoldRemainingSeconds(configuration, elapsedSeconds, activePlan.duration) <= 0.0)
         {
            double remaining = Gr00tHumanoidTrajectoryTools.endpointConfirmationRemainingSeconds(configuration, elapsedSeconds, activePlan.duration);
            updatePlanStatus("Trajectory complete; waiting %.2fs for measured wrist endpoint before continuing".formatted(remaining));
         }
         return Completion.NONE;
      }
      return endpointConverged ? Completion.CONVERGED : Completion.TIMED_OUT;
   }

   void resetForStart()
   {
      acceptedGoal = null;
      acceptedActionChunk = null;
      activePlan = null;
      activePlanStartNanos = -1L;
      pendingPreview = null;
      trajectoryEverPublished = false;
      trajectoryPublished = false;
      lastPublishedHandSample = 0;
      executeAcceptedGoalRequested = controlRobot;
      planStatus = "Waiting for policy actions";
   }

   void clearActiveChunk()
   {
      trajectoryPublished = false;
      acceptedGoal = null;
      acceptedActionChunk = null;
      activePlan = null;
      activePlanStartNanos = -1L;
      lastPublishedHandSample = 0;
      planDuration = Double.NaN;
      worstIKQuality = Double.NaN;
      executeAcceptedGoalRequested = false;
   }

   void discardAcceptedActionChunk()
   {
      acceptedActionChunk = null;
   }

   void resetInitialTransit()
   {
      trajectoryEverPublished = false;
   }

   void setControlRobot(boolean controlRobot)
   {
      boolean wasControlRobot = this.controlRobot;
      this.controlRobot = controlRobot;
      if (controlRobot && !wasControlRobot)
      {
         executeAcceptedGoalRequested = true;
         controlNeck = configuration.controlsNeck();
      }
      if (!controlRobot)
         controlNeck = false;
   }

   boolean isControlRobot()
   {
      return controlRobot;
   }

   void setControlNeck(boolean controlNeck)
   {
      this.controlNeck = controlRobot && configuration.controlsNeck() && controlNeck;
      if (this.controlNeck && activePlan != null)
         publishNeck(activePlan);
   }

   boolean isControlNeck()
   {
      return controlNeck;
   }

   boolean hasActivePlan()
   {
      return activePlan != null;
   }

   Gr00tHumanoidTask.PreviewState getPendingPreview()
   {
      return pendingPreview;
   }

   String getPlanStatus()
   {
      return planStatus;
   }

   void setPlanStatus(String status)
   {
      updatePlanStatus(status);
   }

   Pose3D getAcceptedWristTarget()
   {
      Gr00tHumanoidAction goal = acceptedGoal;
      return goal == null ? null : goal.getWristPose(configuration.armSide());
   }

   double getPlanDuration()
   {
      return planDuration;
   }

   double getWorstIKQuality()
   {
      return worstIKQuality;
   }

   boolean isAllowPoorIKForTesting()
   {
      return allowPoorIKForTesting;
   }

   void setAllowPoorIKForTesting(boolean allowPoorIKForTesting)
   {
      this.allowPoorIKForTesting = allowPoorIKForTesting;
   }

   void recordActionsReceived(int count)
   {
      diagnostics.numberOfActionsReceived.add(count);
   }

   private boolean isCurrentWristNear()
   {
      synchronized (syncedRobot)
      {
         if (!syncedRobot.getDataReceptionTimerSnapshot().isRunning(0.1))
            return false;
         FullHumanoidRobotModel liveRobotModel = syncedRobot.getFullRobotModel();
         FramePose3D currentPose = new FramePose3D(liveRobotModel.getHand(configuration.armSide()).getBodyFixedFrame());
         currentPose.changeFrame(ReferenceFrame.getWorldFrame());
         return Gr00tHumanoidTrajectoryTools.isEndpointConverged(configuration,
                                                                 currentPose,
                                                                 acceptedGoal.getWristPoseReadOnly(configuration.armSide()));
      }
   }

   private Gr00tHumanoidArmPlanner.Plan buildPlan(List<Gr00tHumanoidAction> candidates, boolean initialTransit)
   {
      Gr00tHumanoidArmPlanner.BuildResult result = armPlanner.build(candidates, initialTransit, allowPoorIKForTesting);
      if (result.plan != null)
         return result.plan;
      if (!Double.isNaN(result.rejectionQuality))
         worstIKQuality = result.rejectionQuality;
      updatePlanStatus(result.rejectionReason);
      return null;
   }

   private void installPlan(Gr00tHumanoidArmPlanner.Plan plan)
   {
      activePlan = plan;
      activePlanStartNanos = System.nanoTime();
      lastPublishedHandSample = 0;
      planDuration = plan.duration;
      worstIKQuality = plan.worstIKQuality;
      planStatus = (Gr00tHumanoidArmPlanner.isPoorQuality(plan) ? "TEST OVERRIDE: accepted poor-quality fixed-base plan"
                                                       : "Validated fixed-base plan")
                   + ": %d policy actions / %d trajectory points, %.2fs, worst IK %.4g"
                         .formatted(plan.actionCount, plan.fullJointPositions.length - 1, plan.duration, plan.worstIKQuality);
      statusUpdater.accept(planStatus);
      updatePreview();
   }

   private void rebuildAcceptedPlan()
   {
      Gr00tHumanoidArmPlanner.Plan plan = buildPlan(acceptedActionChunk, !trajectoryEverPublished);
      if (plan != null)
      {
         installPlan(plan);
         publishPlan(plan);
      }
   }

   private void publishPlan(Gr00tHumanoidArmPlanner.Plan plan)
   {
      if (!controlRobot)
         return;
      armTrajectoryPublisher.publish(plan.armMessage);
      trajectoryEverPublished = true;
      trajectoryPublished = true;
      if (controlNeck)
         publishNeck(plan);
      statusUpdater.accept("Published %d GR00T actions as one fixed-base %s-arm trajectory (%.2fs)"
                                 .formatted(plan.actionCount, configuration.armSide().getLowerCaseName(), plan.duration));
   }

   /** Publishes each learned hand sample when its matching arm waypoint becomes due. */
   private void publishDueHandTarget(double elapsed)
   {
      if (!controlRobot || !trajectoryPublished || acceptedActionChunk == null)
         return;
      int dueSample = Gr00tHumanoidTrajectoryTools.latestDueAction(activePlan.trajectoryTimes, elapsed);
      if (dueSample > lastPublishedHandSample)
      {
         boolean published = true;
         for (RobotSide side : RobotSide.values)
         {
            if (configuration.controlsHand(side))
               published &= hands.publishPolicyTargets(side, acceptedActionChunk.get(dueSample - 1).getHandTargets(side));
         }
         if (published)
            lastPublishedHandSample = dueSample;
      }
   }

   private void publishNeck(Gr00tHumanoidArmPlanner.Plan plan)
   {
      if (controlRobot && controlNeck && plan.neckMessage != null)
         neckTrajectoryPublisher.publish(plan.neckMessage);
   }

   private void updatePlanStatus(String status)
   {
      planStatus = status;
      statusUpdater.accept(status);
   }
}
