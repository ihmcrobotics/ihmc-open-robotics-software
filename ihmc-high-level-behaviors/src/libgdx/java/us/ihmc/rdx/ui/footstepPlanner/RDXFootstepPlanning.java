package us.ihmc.rdx.ui.footstepPlanner;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersBasics;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.InitialStanceSide;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.Throttler;

import java.util.ArrayList;

public class RDXFootstepPlanning
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ControllerStatusTracker controllerStatusTracker;
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final AStarBodyPathPlannerParametersBasics bodyPathPlannerParameters;
   private final SwingPlannerParametersBasics swingFootPlannerParameters;
   private final RDXLocomotionParameters locomotionParameters;
   private final MovingReferenceFrame midFeetZUpFrame;
   private final FootstepPlannerLogger footstepPlannerLogger;
   private final ResettableExceptionHandlingExecutorService executor;
   private final Throttler planningThrottler = new Throttler().setFrequency(5.0);
   private final TypedNotification<Pose3DReadOnly> planningRequestNotification = new TypedNotification<>();
   private volatile HeightMapMessage heightMapMessage = null;
   private final FramePose3D midFeetZUpPose = new FramePose3D();
   private final FramePose3D startPose = new FramePose3D();
   /**
    * We create this field so that we can terminate a running plan via
    * a custom termination condition so we don't have to wait
    * for plans to finish when we are going to replan anyway.
    * This increase the response of the control ring plans.
    */
   private boolean terminatePlan = false;
   private final TypedNotification<FootstepPlannerOutput> plannerOutputNotification = new TypedNotification<>();

   public RDXFootstepPlanning(DRCRobotModel robotModel,
                              ROS2SyncedRobotModel syncedRobot,
                              ControllerStatusTracker controllerStatusTracker,
                              RDXLocomotionParameters locomotionParameters,
                              FootstepPlannerParametersBasics footstepPlannerParameters,
                              AStarBodyPathPlannerParametersBasics bodyPathPlannerParameters,
                              SwingPlannerParametersBasics swingFootPlannerParameters)
   {
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;
      this.locomotionParameters = locomotionParameters;
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.bodyPathPlannerParameters = bodyPathPlannerParameters;
      this.swingFootPlannerParameters = swingFootPlannerParameters;

      midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      footstepPlanner.addCustomTerminationCondition(((plannerTime, iterations, bestFinalStep, bestSecondToLastStep, bestPathSize) -> terminatePlan));
      footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);

      executor = MissingThreadTools.newSingleThreadExecutor("FootstepPlanning", true, 1);
   }

   public void update()
   {
      // Throttle the planning submission so we don't plan way too much.
      // We use a notification that we don't check until we're ready to submit.
      // This makes sure the latest planning goal submitted eventually gets
      // planned. It's easy to miss this case and the last submitted plan
      // could get ignored because the throttler wasn't ready yet.
      if (planningThrottler.run())
      {
         if (planningRequestNotification.poll())
         {
            Pose3DReadOnly goalPoseInWorld = planningRequestNotification.read();
            executor.clearQueueAndExecute(() -> planOnAsynchronousThread(goalPoseInWorld, heightMapMessage));
         }
      }
   }

   public void queueAsynchronousPlanning(Pose3DReadOnly goalPoseInWorld)
   {
      // Set termination condition to terminate the running plan as soon as possible,
      // in the case that there is one running.
      terminatePlan = true;

      // Copy the goal pose so we don't modify the sender's copy later
      planningRequestNotification.set(new Pose3D(goalPoseInWorld));
   }

   private void planOnAsynchronousThread(Pose3DReadOnly goalPose, HeightMapMessage heightMapMessage)
   {
      // Set to false as soon as we start, so it can be set to true at any point now.
      terminatePlan = false;

      if (footstepPlanner.isPlanning())
      {
         footstepPlanner.halt();
      }

      footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
      footstepPlanner.getAStarBodyPathPlannerParameters().set(bodyPathPlannerParameters);
      footstepPlanner.getSwingPlannerParameters().set(swingFootPlannerParameters);

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setTimeout(locomotionParameters.getFootstepPlannerTimeout());
      footstepPlannerRequest.setGoalFootPoses(locomotionParameters.getIdealGoalFootstepWidth(), goalPose);

      if (locomotionParameters.getPlanSwingTrajectories())
         footstepPlannerRequest.setSwingPlannerType(SwingPlannerType.MULTI_WAYPOINT_POSITION);
      else
         footstepPlannerRequest.setSwingPlannerType(SwingPlannerType.NONE);

      footstepPlannerRequest.getStartFootPoses().forEach((side, pose3D) ->
      {
         FramePose3DReadOnly soleFramePose;
         if (controllerStatusTracker.getFootstepTracker().getNumberOfIncompleteFootsteps() > 0)
         {
            // We pass in the opposite side because the method returns the footstep on the opposite side
            soleFramePose = controllerStatusTracker.getFootstepTracker().getLastFootstepQueuedOnOppositeSide(side.getOppositeSide());
         }
         else
         {
            soleFramePose = syncedRobot.getFramePoseReadOnly(referenceFrames -> referenceFrames.getSoleFrame(side));
         }
         soleFramePose.get(pose3D);
      });

      if (locomotionParameters.getInitialStanceSide() == InitialStanceSide.LEFT.ordinal())
         footstepPlannerRequest.setRequestedInitialStanceSide(RobotSide.LEFT);
      else if (locomotionParameters.getInitialStanceSide() == InitialStanceSide.RIGHT.ordinal())
         footstepPlannerRequest.setRequestedInitialStanceSide(RobotSide.RIGHT);
      else // AUTO
         footstepPlannerRequest.setRequestedInitialStanceSide(getStanceSideToClosestToGoal(footstepPlannerRequest, goalPose));

      boolean assumeFlatGround = true;
      if (!locomotionParameters.getAssumeFlatGround())
      {
         if (heightMapMessage != null)
         {
            assumeFlatGround = false;
            footstepPlannerRequest.setHeightMapData(HeightMapMessageTools.unpackMessage(heightMapMessage));
         }
      }
      footstepPlannerRequest.setAssumeFlatGround(assumeFlatGround);
      boolean snapGoalSteps = !assumeFlatGround;
      footstepPlannerRequest.setSnapGoalSteps(snapGoalSteps);

      footstepPlannerRequest.setPlanBodyPath(locomotionParameters.getPlanWithBodyPath());

      // If we are not planning the body path,
      // for teleoperation we usually want to stay facing the direction of the goal pose.
      // TODO: Add options and control over this, ideally via the gizmo or context menu
      if (!footstepPlannerRequest.getPlanBodyPath())
      {
         midFeetZUpPose.setToZero(midFeetZUpFrame);
         midFeetZUpPose.changeFrame(ReferenceFrame.getWorldFrame());
         startPose.setToZero(midFeetZUpFrame);
         startPose.changeFrame(ReferenceFrame.getWorldFrame());
         startPose.getOrientation().set(goalPose.getOrientation());
         footstepPlannerRequest.getBodyPathWaypoints().add(midFeetZUpPose);
         footstepPlannerRequest.getBodyPathWaypoints().add(startPose);
         footstepPlannerRequest.getBodyPathWaypoints().add(goalPose);
      }

      footstepPlanner.handleRequest(footstepPlannerRequest);

      // Deep copy because we are handing this off to another thread
      FootstepPlannerOutput output = new FootstepPlannerOutput(footstepPlanner.getOutput());

      LogTools.info("Footstep planner completed with body path {}, footstep planner {}, {} step(s)",
                    output.getBodyPathPlanningResult(),
                    output.getFootstepPlanningResult(),
                    output.getFootstepPlan().getNumberOfSteps());

      ThreadTools.startAThread(() ->
                               {
                                  footstepPlannerLogger.logSession();
                                  FootstepPlannerLogger.deleteOldLogs();
                               }, "FootstepPlanLogAndDeletion");

      boolean plannerFailed = output.getFootstepPlan().getNumberOfSteps() < 1;
      if (plannerFailed)
      {
         FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanner);
         rejectionReasonReport.update();
         ArrayList<Pair<Integer, Double>> rejectionReasonsMessage = new ArrayList<>();
         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            LogTools.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
            rejectionReasonsMessage.add(MutablePair.of(reason == null ? -1 : reason.ordinal(),
                                                       MathTools.roundToSignificantFigures(rejectionPercentage, 3)));
         }
         LogTools.info("Footstep planning failure...");

         // Clears the notification
         plannerOutputNotification.poll();
      }
      else
      {
         plannerOutputNotification.set(output);
      }
   }

   private RobotSide getStanceSideToClosestToGoal(FootstepPlannerRequest request, Pose3DReadOnly goalPose)
   {
      if (request.getStartFootPoses().get(RobotSide.LEFT ).getPosition().distance(goalPose.getPosition())
          <= request.getStartFootPoses().get(RobotSide.RIGHT).getPosition().distance(goalPose.getPosition()))
      {
         return RobotSide.LEFT;
      }
      else
      {
         return RobotSide.RIGHT;
      }
   }

   public void setHeightMapData(HeightMapMessage heightMapMessage)
   {
      this.heightMapMessage = heightMapMessage;
   }

   public TypedNotification<FootstepPlannerOutput> getPlannerOutputNotification()
   {
      return plannerOutputNotification;
   }

   public void destroy()
   {
      executor.destroy();
   }

   public boolean isPlanning()
   {
      return executor.isExecuting();
   }
}
