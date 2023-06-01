package us.ihmc.rdx.ui.footstepPlanner;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
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

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class RDXFootstepPlanning
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final AStarBodyPathPlannerParametersBasics bodyPathPlannerParameters;
   private final SwingPlannerParametersBasics swingFootPlannerParameters;
   private final FootstepPlannerLogger footstepPlannerLogger;
   private final ResettableExceptionHandlingExecutorService executor;
   private boolean isReadyToWalk = false;
   private final Notification plannedNotification = new Notification();

   private final AtomicReference<Pose3DReadOnly> goalPoseReference = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegionsListMessageReference = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsList> planarRegionsListReference = new AtomicReference<>();
   private final AtomicReference<HeightMapMessage> heightMapDataReference = new AtomicReference<>();
   private final AtomicReference<FootstepPlannerOutput> outputReference = new AtomicReference<>();

   private final MovingReferenceFrame midFeetZUpFrame;
   private final FramePose3D midFeetZUpPose = new FramePose3D();
   private final FramePose3D startPose = new FramePose3D();
   private final RDXLocomotionParameters locomotionParameters;

   private final AtomicBoolean hasNewPlanAvailable = new AtomicBoolean(false);
   /**
    * We create this field so that we can terminate a running plan via
    * a custom termination condition so we don't have to wait
    * for plans to finish when we are going to replan anyway.
    * This increase the response of the control ring plans.
    */
   private boolean terminatePlan = false;

   public RDXFootstepPlanning(DRCRobotModel robotModel,
                              ROS2SyncedRobotModel syncedRobot,
                              RDXLocomotionParameters locomotionParameters,
                              FootstepPlannerParametersBasics footstepPlannerParameters,
                              AStarBodyPathPlannerParametersBasics bodyPathPlannerParameters,
                              SwingPlannerParametersBasics swingFootPlannerParameters)
   {
      this.syncedRobot = syncedRobot;
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

   public void planAsync()
   {
      if (goalPoseReference.get() != null)
      {
         // Set termination condition to terminate the running plan as soon as possible,
         // in the case that there is one running.
         terminatePlan = true;

         executor.clearQueueAndExecute(this::plan);
      }
   }

   private void plan()
   {
      // Set to false as soon as we start, so it can be set to true at any point now.
      terminatePlan = false;

      if (footstepPlanner.isPlanning())
      {
         footstepPlanner.halt();
      }

      PlanarRegionsListMessage planarRegionsListMessage = planarRegionsListMessageReference.get();
      HeightMapMessage heightMapMessage = heightMapDataReference.get();
      PlanarRegionsList planarRegionsList = planarRegionsListReference.get();
      Pose3DReadOnly goalPose = goalPoseReference.getAndSet(null);
      if (goalPose == null)
         return;

      footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
      footstepPlanner.getAStarBodyPathPlannerParameters().set(bodyPathPlannerParameters);
      footstepPlanner.getSwingPlannerParameters().set(swingFootPlannerParameters);

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();

      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), goalPose);
      setStanceSideToClosestToGoal(footstepPlannerRequest, goalPose);

      footstepPlannerRequest.setSwingPlannerType(SwingPlannerType.MULTI_WAYPOINT_POSITION);
      footstepPlannerRequest.getStartFootPoses().forEach((side, pose3D) ->
      {
         FramePose3DReadOnly soleFramePose = syncedRobot.getFramePoseReadOnly(referenceFrames -> referenceFrames.getSoleFrame(side));
         soleFramePose.get(pose3D);
      });

      boolean assumeFlatGround = true;
      if (!locomotionParameters.getAssumeFlatGround())
      {
         if (heightMapMessage != null)
         {
            assumeFlatGround = false;
            footstepPlannerRequest.setHeightMapData(HeightMapMessageTools.unpackMessage(heightMapMessage));
         }
         if (planarRegionsListMessage != null)
         {
            footstepPlannerRequest.setPlanarRegionsList(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
            assumeFlatGround = false;
         }
         if (planarRegionsList != null)
         {
            footstepPlannerRequest.setPlanarRegionsList(planarRegionsList);
            assumeFlatGround = false;
         }
      }
      footstepPlannerRequest.setAssumeFlatGround(assumeFlatGround);

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

      // TODO: Set start footholds!!
      //      request.setTimeout(lookAndStepParameters.getFootstepPlannerTimeoutWhileStopped());

      FootstepPlannerOutput output = footstepPlanner.getOutput();

      footstepPlanner.handleRequest(footstepPlannerRequest);
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
            rejectionReasonsMessage.add(MutablePair.of(reason.ordinal(), MathTools.roundToSignificantFigures(rejectionPercentage, 3)));
         }
         LogTools.info("Footstep planning failure...");

         outputReference.set(null);
      }
      else
      {
         outputReference.set(output);
      }

      isReadyToWalk = !plannerFailed;
      plannedNotification.set();
      hasNewPlanAvailable.set(isReadyToWalk);
   }

   private void setStanceSideToClosestToGoal(FootstepPlannerRequest footstepPlannerRequest, Pose3DReadOnly goalPose)
   {
      RobotSide stanceSide;
      if (footstepPlannerRequest.getStartFootPoses().get(RobotSide.LEFT ).getPosition().distance(goalPose.getPosition())
       <= footstepPlannerRequest.getStartFootPoses().get(RobotSide.RIGHT).getPosition().distance(goalPose.getPosition()))
      {
         stanceSide = RobotSide.LEFT;
      }
      else
      {
         stanceSide = RobotSide.RIGHT;
      }

      footstepPlannerRequest.setRequestedInitialStanceSide(stanceSide);
   }

   public boolean pollHasNewPlanAvailable()
   {
      return hasNewPlanAvailable.getAndSet(false);
   }

   public FootstepPlannerOutput pollOutput()
   {
      return outputReference.getAndSet(null);
   }

   public void setMidFeetGoalPose(Pose3DReadOnly midFeetGoalPose)
   {
      this.goalPoseReference.set(midFeetGoalPose);
   }

   public void setPlanarRegionsListMessage(PlanarRegionsListMessage planarRegionsListMessage)
   {
      this.planarRegionsListMessageReference.set(planarRegionsListMessage);
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsListReference.set(planarRegionsList);
   }

   public void setHeightMapData(HeightMapMessage heightMapMessage)
   {
      this.heightMapDataReference.set(heightMapMessage);
   }

   public boolean isReadyToWalk()
   {
      return isReadyToWalk;
   }

   public void setReadyToWalk(boolean readyToWalk)
   {
      isReadyToWalk = readyToWalk;
   }

   public Notification getPlannedNotification()
   {
      return plannedNotification;
   }

   public void destroy()
   {
      executor.destroy();
   }
}
