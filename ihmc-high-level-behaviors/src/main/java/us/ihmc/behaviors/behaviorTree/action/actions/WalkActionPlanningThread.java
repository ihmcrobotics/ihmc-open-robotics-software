package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.WalkActionDefinitionMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.InitialStanceSide;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WalkActionPlanningThread
{
   private final boolean isPreviewPlanner;
   private final WalkActionState state;
   private final WalkActionDefinition definition;
   private long started = 0;
   private long completed = 0;
   private final FootstepPlanningModule footstepPlanner;
   private final SideDependentList<FramePose3D> startFootPoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalFootPoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final FramePose3D startMidFeetPose = new FramePose3D();
   private final FramePose3D startTurnedToMatchGoalFacing = new FramePose3D();
   private final FramePose3D goalMidFeetPose = new FramePose3D();
   private FootstepPlan result;
   private final TypedNotification<FootstepPlan> resultNotification = new TypedNotification<>();
   private final TerrainMapData terrainMapData;

   public WalkActionPlanningThread(boolean isPreviewPlanner, WalkActionState state, WalkActionDefinition definition, TerrainMapData terrainMapData)
   {
      this.isPreviewPlanner = isPreviewPlanner;
      this.state = state;
      this.definition = definition;
      this.terrainMapData = terrainMapData;
      footstepPlanner = new FootstepPlanningModule();
   }

   public void triggerPlan(ROS2SyncedRobotModel syncedRobot, SideDependentList<FramePose3D> liveGoalFeetPoses)
   {
      ++started;

      for (RobotSide side : RobotSide.values)
      {
         startFootPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
         goalFootPoses.get(side).set(liveGoalFeetPoses.get(side));
      }

      Thread thread = new Thread(() ->
      {
         try
         {
            plan(started);
         }
         catch (Throwable throwable)
         {
            LogTools.error(throwable.getMessage());
            throwable.printStackTrace();
         }
      }, getClass().getSimpleName() + started);
      thread.start();
   }

   public boolean planningComplete()
   {
      return completed == started;
   }

   public FootstepPlan getResult()
   {
      return result;
   }

   public TypedNotification<FootstepPlan> getResultNotification()
   {
      return resultNotification;
   }

   private void plan(long sequenceID)
   {
      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      // TODO: Set start footholds
      for (RobotSide side : RobotSide.values)
      {
         footstepPlannerRequest.setStartFootPose(side, startFootPoses.get(side));
         footstepPlannerRequest.setGoalFootPose(side, goalFootPoses.get(side));
      }

      if (definition.getPlannerInitialStanceSide().getValue() == InitialStanceSide.LEFT)
         footstepPlannerRequest.setRequestedInitialStanceSide(RobotSide.LEFT);
      else if (definition.getPlannerInitialStanceSide().getValue() == InitialStanceSide.RIGHT)
         footstepPlannerRequest.setRequestedInitialStanceSide(RobotSide.RIGHT);
      else // AUTO, swing the foot furthest from the goal first
      {
         double leftStartToGoal = goalFootPoses.get(RobotSide.LEFT).getPositionDistance(startFootPoses.get(RobotSide.LEFT));
         double rightStartToGoal = goalFootPoses.get(RobotSide.RIGHT).getPositionDistance(startFootPoses.get(RobotSide.RIGHT));
         footstepPlannerRequest.setRequestedInitialStanceSide(leftStartToGoal < rightStartToGoal ? RobotSide.LEFT : RobotSide.RIGHT);
      }

      footstepPlannerRequest.setPerformAStarSearch(definition.getPlannerType().getValue() == WalkActionDefinitionMessage.A_STAR);
      footstepPlanner.getFootstepPlannerParameters().set(definition.getPlannerParametersReadOnly());
      boolean planWithBodyPath = definition.getPlannerPlanWithBodyPath().getValue();
      footstepPlannerRequest.setPlanBodyPath(planWithBodyPath);
      if (planWithBodyPath && terrainMapData != null)
      {
         footstepPlannerRequest.setTerrainMapData(terrainMapData);
      }
      footstepPlannerRequest.setAssumeFlatGround(planWithBodyPath);

      if (definition.getPlannerWalkWithGoalOrientation().getValue())
      {
         // At beginning, first turn in place to face the direction that the goal stance faces
         startMidFeetPose.interpolate(startFootPoses.get(RobotSide.LEFT), startFootPoses.get(RobotSide.RIGHT), 0.5);
         goalMidFeetPose.interpolate(goalFootPoses.get(RobotSide.LEFT), goalFootPoses.get(RobotSide.RIGHT), 0.5);
         startTurnedToMatchGoalFacing.set(startMidFeetPose);
         startTurnedToMatchGoalFacing.getOrientation().set(goalMidFeetPose.getOrientation());
         footstepPlannerRequest.getBodyPathWaypoints().add(startMidFeetPose);
         footstepPlannerRequest.getBodyPathWaypoints().add(startTurnedToMatchGoalFacing);
         footstepPlannerRequest.getBodyPathWaypoints().add(goalMidFeetPose);
      }

      if (!isPreviewPlanner)
         state.getLogger().info("Planning footsteps...");

      FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest, isPreviewPlanner);
      boolean plannerBusy = footstepPlannerOutput == null;
      boolean foundSolution = !plannerBusy && footstepPlannerOutput.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION;

      while (plannerBusy) // Retry until planner is free
      {
         if (isPreviewPlanner)
            return; // We don't need to do anything else here, another one will get scheduled

         state.getLogger().info("Planner already running. Trying again in 1 s... (Planner timeout is %.1f s)".formatted(footstepPlannerRequest.getTimeout()));

         ThreadTools.parkAtLeast(1.0);
         footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest, isPreviewPlanner);
         plannerBusy = footstepPlannerOutput == null;
         foundSolution = !plannerBusy && footstepPlannerOutput.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION;
      }

      if (foundSolution)
      {
         if (!isPreviewPlanner)
            state.getLogger().info("Footstep planner completed with {}, {} step(s)",
                                   footstepPlannerOutput.getFootstepPlanningResult(),
                                   footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());
      }
      else
      {
         FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanner);
         rejectionReasonReport.update();
         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            state.getLogger().info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
         }
         state.getLogger().info("Footstep planning failed with {}, {} step(s)", footstepPlannerOutput.getFootstepPlanningResult(),
                                                                                footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());
      }

      // Copy of the output to be safe & use clean empty plan when no solution found
      FootstepPlan modifiedFootstepPlan = new FootstepPlan(foundSolution ? footstepPlannerOutput.getFootstepPlan() : new FootstepPlan());
      for (int i = 0; i < modifiedFootstepPlan.getNumberOfSteps(); i++)
      {
         if (i == 0)
            modifiedFootstepPlan.getFootstep(i).setTransferDuration(definition.getTransferDuration() / 2.0);
         else
            modifiedFootstepPlan.getFootstep(i).setTransferDuration(definition.getTransferDuration());

         modifiedFootstepPlan.getFootstep(i).setSwingDuration(definition.getSwingDuration());
      }

      if (!isPreviewPlanner)
      {
         FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
         footstepPlannerLogger.logSession();
         FootstepPlannerLogger.deleteOldLogs();
      }

      // Prevent an ealier plan from overwriting a later one
      if (sequenceID > completed)
      {
         result = modifiedFootstepPlan;
         completed = sequenceID;
         resultNotification.set(result);
      }
   }
}
