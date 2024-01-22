package us.ihmc.behaviors.sequence.actions;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class WalkActionExecutor extends ActionNodeExecutor<WalkActionState, WalkActionDefinition>
{
   private final WalkActionState state;
   private final FootstepPlanActionExecutorBasics footstepPlanExecutorBasics;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final ResettableExceptionHandlingExecutorService footstepPlanningThread = MissingThreadTools.newSingleThreadExecutor("FootstepPlanning", true, 1);
   private final TypedNotification<FootstepPlan> footstepPlanNotification = new TypedNotification<>();
   private final SideDependentList<FramePose3D> liveGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> startFootPosesForThread = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalFootPosesForThread = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   public WalkActionExecutor(long id,
                             CRDTInfo crdtInfo,
                             WorkspaceResourceDirectory saveFileDirectory,
                             ROS2ControllerHelper ros2ControllerHelper,
                             ROS2SyncedRobotModel syncedRobot,
                             WalkingFootstepTracker footstepTracker,
                             FootstepPlanningModule footstepPlanner,
                             FootstepPlannerParametersBasics footstepPlannerParameters,
                             WalkingControllerParameters walkingControllerParameters,
                             ReferenceFrameLibrary referenceFrameLibrary,
                             SceneGraph sceneGraph)
   {
      super(new WalkActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();

      this.syncedRobot = syncedRobot;
      this.footstepPlanner = footstepPlanner;
      this.footstepPlannerParameters = footstepPlannerParameters;

      footstepPlanExecutorBasics = new FootstepPlanActionExecutorBasics(state.getBasics(),
                                                                        getDefinition().getBasics(),
                                                                        ros2ControllerHelper,
                                                                        syncedRobot,
                                                                        footstepTracker,
                                                                        walkingControllerParameters);
   }

   @Override
   public void update()
   {
      super.update();

      footstepPlanExecutorBasics.update();

      state.setCanExecute(state.getGoalFrame().isChildOfWorld());

      if (state.getGoalFrame().isChildOfWorld())
      {
         for (RobotSide side : RobotSide.values)
         {
            liveGoalFeetPoses.get(side).setIncludingFrame(state.getGoalFrame().getReferenceFrame(),
                                                          getDefinition().getGoalFootstepToGoalTransform(side).getValueReadOnly());
            liveGoalFeetPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         }
      }
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      if (state.getGoalFrame().isChildOfWorld())
      {
         state.getExecutionState().setValue(WalkActionExecutionState.TRIGGERED);
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      switch (state.getExecutionState().getValue())
      {
         case TRIGGERED ->
         {
            state.setIsExecuting(true);
            state.getExecutionState().setValue(WalkActionExecutionState.FOOTSTEP_PLANNING);

            for (RobotSide side : RobotSide.values)
            {
               startFootPosesForThread.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
               goalFootPosesForThread.get(side).set(liveGoalFeetPoses.get(side));
            }

            footstepPlanNotification.poll(); // Make sure it's cleared
            footstepPlanningThread.execute(() ->
            {
               footstepPlannerParameters.setFinalTurnProximity(1.0);

               FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
               footstepPlannerRequest.setPlanBodyPath(false);
               footstepPlannerRequest.setStartFootPoses(startFootPosesForThread.get(RobotSide.LEFT), startFootPosesForThread.get(RobotSide.RIGHT));
               // TODO: Set start footholds!!
               for (RobotSide side : RobotSide.values)
               {
                  footstepPlannerRequest.setGoalFootPose(side, goalFootPosesForThread.get(side));
               }

               //      footstepPlannerRequest.setPlanarRegionsList(...);
               footstepPlannerRequest.setAssumeFlatGround(true); // FIXME Assuming flat ground

               footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
               double idealFootstepLength = 0.5;
               footstepPlanner.getFootstepPlannerParameters().setIdealFootstepLength(idealFootstepLength);
               footstepPlanner.getFootstepPlannerParameters().setMaximumStepReach(idealFootstepLength);
               LogTools.info("Planning footsteps...");
               FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
               FootstepPlan footstepPlan = footstepPlannerOutput.getFootstepPlan();
               LogTools.info("Footstep planner completed with {}, {} step(s)",
                             footstepPlannerOutput.getFootstepPlanningResult(),
                             footstepPlan.getNumberOfSteps());

               if (footstepPlan.getNumberOfSteps() < 1) // failed
               {
                  FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanner);
                  rejectionReasonReport.update();
                  for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
                  {
                     double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
                     LogTools.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
                  }
                  LogTools.info("Footstep planning failure...");
                  footstepPlanNotification.set(new FootstepPlan());
               }
               else
               {
                  for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
                  {
                     if (i == 0 || i == footstepPlan.getNumberOfSteps() - 1)
                        footstepPlan.getFootstep(i).setTransferDuration(getDefinition().getBasics().getTransferDuration() / 2.0);
                     else
                        footstepPlan.getFootstep(i).setTransferDuration(getDefinition().getBasics().getTransferDuration());

                     footstepPlan.getFootstep(i).setSwingDuration(getDefinition().getBasics().getSwingDuration());
                  }
                  footstepPlanNotification.set(new FootstepPlan(footstepPlan)); // Copy of the output to be safe
               }

               FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
               footstepPlannerLogger.logSession();
               FootstepPlannerLogger.deleteOldLogs();
            }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }
         case FOOTSTEP_PLANNING ->
         {
            // TODO: Maybe report planning elapsed time or something
            if (footstepPlanNotification.poll())
            {
               FootstepPlan footstepPlan = footstepPlanNotification.read();
               if (footstepPlan.isEmpty())
               {
                  state.getExecutionState().setValue(WalkActionExecutionState.PLANNING_FAILED);
               }
               else
               {
                  state.getExecutionState().setValue(WalkActionExecutionState.PLANNING_SUCCEEDED);
                  footstepPlanExecutorBasics.setFootstepPlanToExecute(footstepPlan);
               }
            }
         }
         case PLANNING_FAILED ->
         {
            state.setIsExecuting(false);
            state.setFailed(true);
            state.getExecutionState().setValue(WalkActionExecutionState.PLAN_EXECUTION_COMPLETE);
         }
         case PLANNING_SUCCEEDED ->
         {
            footstepPlanExecutorBasics.triggerActionExecution();
            state.getExecutionState().setValue(WalkActionExecutionState.PLAN_COMMANDED);
         }
         case PLAN_COMMANDED ->
         {
            footstepPlanExecutorBasics.updateCurrentlyExecuting(this);
            if (!state.getIsExecuting())
            {
               state.getExecutionState().setValue(WalkActionExecutionState.PLAN_EXECUTION_COMPLETE);
            }
         }
      }
   }
}
