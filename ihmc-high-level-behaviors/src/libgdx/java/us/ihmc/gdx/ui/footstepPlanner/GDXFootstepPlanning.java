package us.ihmc.gdx.ui.footstepPlanner;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
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

public class GDXFootstepPlanning
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerLogger footstepPlannerLogger;
   private final FootstepPlannerRequest request;
   private final FootstepPlannerOutput output;
   private final FramePose3D midFeetGoalPose = new FramePose3D();
   private final ResettableExceptionHandlingExecutorService executor;
   private boolean isReadyToWalk = false;
   private final Throttler throttler = new Throttler();
   private final Notification plannedNotification = new Notification();

   public GDXFootstepPlanning(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      footstepPlannerParameters = footstepPlanner.getFootstepPlannerParameters();
      request = new FootstepPlannerRequest();
      output = footstepPlanner.getOutput();
      footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);

      executor = MissingThreadTools.newSingleThreadExecutor("FootstepPlanning", true, 1);
   }

   public void setGoalFootPosesFromMidFeetPose()
   {
      request.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), midFeetGoalPose);
   }

   public void updateMidFeetGoalPose()
   {
      midFeetGoalPose.set(request.getGoalFootPoses().get(RobotSide.LEFT));
      midFeetGoalPose.interpolate(request.getGoalFootPoses().get(RobotSide.RIGHT), 0.5);
   }

   public void setStanceSideToClosestToGoal()
   {
      RobotSide stanceSide;
      if (request.getStartFootPoses().get(RobotSide.LEFT ).getPosition().distance(midFeetGoalPose.getPosition())
          <= request.getStartFootPoses().get(RobotSide.RIGHT).getPosition().distance(midFeetGoalPose.getPosition()))
      {
         stanceSide = RobotSide.LEFT;
      }
      else
      {
         stanceSide = RobotSide.RIGHT;
      }

      request.setPlanBodyPath(false);
      request.setRequestedInitialStanceSide(stanceSide);
   }

   private void planAsync()
   {
      executor.clearQueueAndExecute(this::plan);
   }

   /**
    * TODO: Make this private and get async working
    */
   public void plan()
   {
      request.getStartFootPoses().forEach((side, pose3D) ->
      {
         FramePose3DReadOnly soleFramePose = syncedRobot.getFramePoseReadOnly(referenceFrames -> referenceFrames.getSoleFrame(side));
         soleFramePose.get(pose3D);
      });

      request.setPlanBodyPath(false);
      // TODO: Set start footholds!!
      //      request.setPlanarRegionsList(...);
      request.setAssumeFlatGround(true); // FIXME Assuming flat ground
      //      request.setTimeout(lookAndStepParameters.getFootstepPlannerTimeoutWhileStopped());
      //      request.setSwingPlannerType(swingPlannerType);
      //      request.setSnapGoalSteps(true);

      LogTools.info("Stance side: {}", request.getRequestedInitialStanceSide().name());
      LogTools.info("Planning footsteps...");
      footstepPlanner.handleRequest(request);
      LogTools.info("Footstep planner completed with {}, {} step(s)",
                    output.getFootstepPlanningResult(),
                    output.getFootstepPlan().getNumberOfSteps());

      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(50), "FootstepPlanLogDeletion");

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
      }

      isReadyToWalk = !plannerFailed;
      plannedNotification.set();
   }

   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlannerParameters;
   }

   public FootstepPlannerOutput getOutput()
   {
      return output;
   }

   public FramePose3D getMidFeetGoalPose()
   {
      return midFeetGoalPose;
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
