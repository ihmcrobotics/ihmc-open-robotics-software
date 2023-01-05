package us.ihmc.rdx.ui.footstepPlanner;

import com.kitfox.svg.A;
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
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.Throttler;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class RDXFootstepPlanning
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerLogger footstepPlannerLogger;
   private final FootstepPlannerRequest request;
   private final ResettableExceptionHandlingExecutorService executor;
   private boolean isReadyToWalk = false;
   private final Throttler throttler = new Throttler();
   private final Notification plannedNotification = new Notification();

   private final AtomicReference<Pose3DReadOnly> goalPoseReference = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegionsReference = new AtomicReference<>();
   private final AtomicReference<HeightMapMessage> heightMapDataReference = new AtomicReference<>();
   private final AtomicReference<FootstepPlannerOutput> outputReference = new AtomicReference<>();

   private final AtomicBoolean hasNewPlanAvailable = new AtomicBoolean(false);

   public RDXFootstepPlanning(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      footstepPlannerParameters = footstepPlanner.getFootstepPlannerParameters();
      request = new FootstepPlannerRequest();
      footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);

      executor = MissingThreadTools.newSingleThreadExecutor("FootstepPlanning", true, 1);
   }

   public void planAsync()
   {
      executor.clearQueueAndExecute(this::plan);
   }

   private void setGoalFootPosesFromMidFeetPose(Pose3DReadOnly goalPose)
   {
      request.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), goalPose);
   }

   private void setStanceSideToClosestToGoal(Pose3DReadOnly goalPose)
   {
      RobotSide stanceSide;
      if (request.getStartFootPoses().get(RobotSide.LEFT ).getPosition().distance(goalPose.getPosition())
          <= request.getStartFootPoses().get(RobotSide.RIGHT).getPosition().distance(goalPose.getPosition()))
      {
         stanceSide = RobotSide.LEFT;
      }
      else
      {
         stanceSide = RobotSide.RIGHT;
      }

      request.setRequestedInitialStanceSide(stanceSide);
   }
   
   private void plan()
   {
      if (footstepPlanner.isPlanning())
         footstepPlanner.halt();

      PlanarRegionsListMessage planarRegionsListMessage = planarRegionsReference.get();
      HeightMapMessage heightMapMessage = heightMapDataReference.get();
      Pose3DReadOnly goalPose = goalPoseReference.getAndSet(null);
      if (goalPose == null)
         return;

      setGoalFootPosesFromMidFeetPose(goalPose);
      setStanceSideToClosestToGoal(goalPose);

      request.getStartFootPoses().forEach((side, pose3D) ->
      {
         FramePose3DReadOnly soleFramePose = syncedRobot.getFramePoseReadOnly(referenceFrames -> referenceFrames.getSoleFrame(side));
         soleFramePose.get(pose3D);
      });

      boolean assumeFlatGround = true;
      if (heightMapMessage != null)
      {
         assumeFlatGround = false;
         request.setHeightMapMessage(heightMapMessage);
      }
      if (planarRegionsListMessage != null)
      {
         request.setPlanarRegionsList(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
         assumeFlatGround = false;
      }

      request.setPlanBodyPath(false);
      // TODO: Set start footholds!!
      //      request.setPlanarRegionsList(...);
      request.setAssumeFlatGround(assumeFlatGround);
      //      request.setTimeout(lookAndStepParameters.getFootstepPlannerTimeoutWhileStopped());
      //      request.setSwingPlannerType(swingPlannerType);
      //      request.setSnapGoalSteps(true);

      FootstepPlannerOutput output = footstepPlanner.getOutput();

      LogTools.info("Stance side: {}", request.getRequestedInitialStanceSide().name());
      LogTools.info("Planning footsteps...");
      footstepPlanner.handleRequest(request);
      LogTools.info("Footstep planner completed with {}, {} step(s)",
                    output.getFootstepPlanningResult(),
                    output.getFootstepPlan().getNumberOfSteps());

      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(), "FootstepPlanLogDeletion");

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

   public boolean pollHasNewPlanAvailable()
   {
      return hasNewPlanAvailable.getAndSet(false);
   }

   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlannerParameters;
   }

   public FootstepPlannerOutput pollOutput()
   {
      return outputReference.getAndSet(null);
   }

   public void setMidFeetGoalPose(Pose3DReadOnly midFeetGoalPose)
   {
      this.goalPoseReference.set(midFeetGoalPose);
   }

   public void setPlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      this.planarRegionsReference.set(planarRegionsListMessage);
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
}
