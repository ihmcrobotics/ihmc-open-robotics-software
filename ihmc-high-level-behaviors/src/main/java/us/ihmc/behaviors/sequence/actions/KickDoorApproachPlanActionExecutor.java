package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.TaskspaceTrajectoryTrackingErrorCalculator;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.donkeyKick.KickDynamicPlanner;
import us.ihmc.commonWalkingControlModules.donkeyKick.KickInputParameters;
import us.ihmc.commonWalkingControlModules.donkeyKick.KickParameters;
import us.ihmc.commons.Conversions;
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
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.UUID;

public class KickDoorApproachPlanActionExecutor extends ActionNodeExecutor<KickDoorApproachPlanActionState, KickDoorApproachPlanActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final KickDoorApproachPlanActionState state;
   private final KickDoorApproachPlanActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ControllerStatusTracker controllerStatusTracker;
   private final WalkingControllerParameters walkingControllerParameters;
   private final SideDependentList<FramePose3D> commandedGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> syncedFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<Integer> indexOfLastFoot = new SideDependentList<>();
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private double nominalExecutionDuration;
   private final SideDependentList<TaskspaceTrajectoryTrackingErrorCalculator> trackingCalculators = new SideDependentList<>(
         TaskspaceTrajectoryTrackingErrorCalculator::new);
   private final FootstepPlan footstepPlanToExecute = new FootstepPlan();
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final ResettableExceptionHandlingExecutorService footstepPlanningThread = MissingThreadTools.newSingleThreadExecutor("FootstepPlanning", true, 1);
   private final TypedNotification<FootstepPlan> footstepPlanNotification = new TypedNotification<>();
   private final SideDependentList<FramePose3D> liveGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final FramePose3D liveKickPose = new FramePose3D();
   private final SideDependentList<FramePose3D> startFootPosesForThread = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalFootPosesForThread = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private final KickParameters kickParameters = new KickParameters();
   private final KickInputParameters kickInputParameters = new KickInputParameters(null);
   private final SideDependentList<PoseReferenceFrame> soleFramesForPlanning = new SideDependentList<>();
   private final PoseReferenceFrame centerOfMassControlFrameForPlanning = new PoseReferenceFrame("CenterOfMassControlFrameForPlanning",
                                                                                                 ReferenceFrame.getWorldFrame());
   private static final double gravityZ = 9.81; // This needs to be positive for the planner to work
   private final KickDynamicPlanner kickDynamicPlanner;

   private final DetachableReferenceFrame doorHandleFrame;
   private ReferenceFrame worldFrame;

   public KickDoorApproachPlanActionExecutor(long id,
                                             CRDTInfo crdtInfo,
                                             WorkspaceResourceDirectory saveFileDirectory,
                                             ROS2ControllerHelper ros2ControllerHelper,
                                             ROS2SyncedRobotModel syncedRobot,
                                             ControllerStatusTracker controllerStatusTracker,
                                             ReferenceFrameLibrary referenceFrameLibrary,
                                             WalkingControllerParameters walkingControllerParameters,
                                             FootstepPlanningModule footstepPlanner,
                                             FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      super(new KickDoorApproachPlanActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      this.referenceFrameLibrary = referenceFrameLibrary;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;
      this.walkingControllerParameters = walkingControllerParameters;
      this.footstepPlanner = footstepPlanner;
      this.footstepPlannerParameters = footstepPlannerParameters;

      doorHandleFrame = state.getDoorHandleFrame();
      doorHandleFrame.update(definition.getParentFrameName());
      worldFrame = doorHandleFrame.getReferenceFrame().getRootFrame();

      for (RobotSide robotSide : RobotSide.values)
      {
         PoseReferenceFrame soleFrameForPlanning = new PoseReferenceFrame(robotSide.getLowerCaseName() + "SoleFrameForPlanning",
                                                                          ReferenceFrame.getWorldFrame());
         soleFramesForPlanning.put(robotSide, soleFrameForPlanning);
      }

      this.kickDynamicPlanner = new KickDynamicPlanner(kickParameters,
                                                       soleFramesForPlanning,
                                                       centerOfMassControlFrameForPlanning,
                                                       gravityZ,
                                                       syncedRobot.getFullRobotModel().getTotalMass(),
                                                       null);
   }

   @Override
   public void update()
   {
      super.update();

      worldFrame = doorHandleFrame.getReferenceFrame().getRootFrame();

      boolean invalidDefinition = false;

      double kickImpulse = definition.getKickImpulse().getValue();
      double kickHeight = definition.getKickHeight().getValue();
      double kickTargetDistance = definition.getKickTargetDistance().getValue();
      double prekickWeightDistribution = definition.getPrekickWeightDistribution().getValue();
      double kickStanceWidth = definition.getStanceFootWidth().getValue();
      double kickOffsetFromHandle = definition.getHorizontalDistanceFromHandle().getValue();

      if (Double.isNaN(kickImpulse) || kickImpulse < 0.0)
      {
         state.getLogger().error("Kick impulse must be greater than 0.0");
         invalidDefinition = true;
      }

      if (Double.isNaN(kickTargetDistance) || kickTargetDistance < 0.0)
      {
         state.getLogger().error("Kick target distance must be greater than 0.0");
         invalidDefinition = true;
      }

      if (Double.isNaN(prekickWeightDistribution) || prekickWeightDistribution < 0.4)
      {
         state.getLogger().error("Pre kick weight distribution must be greater than 0.4");
         invalidDefinition = true;
      }

      if (Double.isNaN(kickStanceWidth) || kickStanceWidth < 0.15)
      {
         state.getLogger().error("Kick stance width must be greater than 0.15");
         invalidDefinition = true;
      }

      if (Double.isNaN(kickOffsetFromHandle) || kickOffsetFromHandle < 0.0)
      {
         state.getLogger().error("Kick offset from handle must be greater than 0.0");
         invalidDefinition = true;
      }

      state.setCanExecute(state.areFramesInWorld() && !invalidDefinition);
      computeGoalFootPosesForKick();

      state.getLeftFootGoalPose().getValue().set(liveGoalFeetPoses.get(RobotSide.LEFT));
      state.getRightFootGoalPose().getValue().set(liveGoalFeetPoses.get(RobotSide.RIGHT));
      state.getKickGoalPose().getValue().set(liveKickPose);

      for (RobotSide side : RobotSide.values)
      {
         trackingCalculators.get(side).update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
      }

      if (state.getCanExecute())
      {
      }
   }

   private void computeGoalFootPosesForKick()
   {
      // update the input parameters from the definition
      kickInputParameters.setKickImpulse(definition.getKickImpulse().getValue());
      kickInputParameters.setKickTargetDistance(definition.getKickTargetDistance().getValue());
      kickInputParameters.setPrekickWeightDistribution(definition.getPrekickWeightDistribution().getValue());
      kickInputParameters.setKickFootSide(definition.getKickSide().getValue());

      computeSquaredUpGoalFootPosesFromDoorHandle();

      // call compute here to get the initial kick foot pose
      kickDynamicPlanner.compute(kickInputParameters);

      updateGoalPosesFromDynamicPlanner();
   }

   private void computeSquaredUpGoalFootPosesFromDoorHandle()
   {
      RobotSide kickSide = definition.getKickSide().getValue();

      FramePose3D preKickFootPose = new FramePose3D();
      preKickFootPose.setToZero(doorHandleFrame.getReferenceFrame());
      preKickFootPose.setX(definition.getKickTargetDistance().getValue());
      preKickFootPose.setY(-kickSide.negateIfRightSide(definition.getHorizontalDistanceFromHandle().getValue()));
      preKickFootPose.changeFrame(worldFrame);
      preKickFootPose.getPosition().setZ(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getZ());


      FramePose3D kickPose = new FramePose3D(preKickFootPose);
      kickPose.changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      kickPose.setZ(definition.getKickHeight().getValue());
      kickPose.changeFrame(doorHandleFrame.getReferenceFrame());
      kickPose.setX(0.0);
      kickPose.appendPitchRotation(Math.PI / 2.0);
      kickPose.changeFrame(worldFrame);

      FramePose3D stanceFootPose = new FramePose3D();
      stanceFootPose.setToZero(doorHandleFrame.getReferenceFrame());
      stanceFootPose.setX(definition.getKickTargetDistance().getValue());
      stanceFootPose.setY(-kickSide.negateIfRightSide(definition.getHorizontalDistanceFromHandle().getValue() + definition.getStanceFootWidth().getValue()));
      stanceFootPose.changeFrame(worldFrame);
      stanceFootPose.getPosition().setZ(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getZ());

      FramePose3D centerOfMassPose = new FramePose3D(worldFrame);
      centerOfMassPose.interpolate(preKickFootPose, stanceFootPose, 0.5);

      soleFramesForPlanning.get(kickSide).setPoseAndUpdate(preKickFootPose);
      soleFramesForPlanning.get(kickSide.getOppositeSide()).setPoseAndUpdate(stanceFootPose);

      liveKickPose.set(kickPose);

      centerOfMassControlFrameForPlanning.setPoseAndUpdate(centerOfMassPose);
   }

   private void updateGoalPosesFromDynamicPlanner()
   {
      RobotSide kickSide = definition.getKickSide().getValue();
      RobotSide stanceSide = kickSide.getOppositeSide();

      FramePose3D stanceGoalPose = new FramePose3D(soleFramesForPlanning.get(stanceSide));
      stanceGoalPose.changeFrame(ReferenceFrame.getWorldFrame());

      liveGoalFeetPoses.get(stanceSide).setIncludingFrame(stanceGoalPose);

      FramePose3D kickStartPose = new FramePose3D(soleFramesForPlanning.get(kickSide));
      kickStartPose.changeFrame(ReferenceFrame.getWorldFrame());
      kickStartPose.getPosition().set(kickDynamicPlanner.getDesiredSwingFootStartNominal());

      liveGoalFeetPoses.get(kickSide).setIncludingFrame(kickStartPose);
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      // Reset state
      state.setTotalNumberOfFootsteps(0);
      state.setNumberOfIncompleteFootsteps(0);
      for (RobotSide side : RobotSide.values)
      {
         state.getCurrentFootPoses().get(side).getValue().set(syncedFeetPoses.get(side));
         state.getDesiredFootPoses().get(side).getValue().clear();
      }

      state.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);

      if (state.areFramesInWorld())
      {
         startFootstepPlanningAsync();
         state.getExecutionState().setValue(FootstepPlanActionExecutionState.FOOTSTEP_PLANNING);
      }
      else
      {
         state.getLogger().error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      switch (state.getExecutionState().getValue())
      {
         case FOOTSTEP_PLANNING ->
         {
            state.setIsExecuting(true);
            // TODO: Maybe report planning elapsed time or something
            if (footstepPlanNotification.poll())
            {
               footstepPlanToExecute.clear();
               footstepPlanToExecute.set(footstepPlanNotification.read());
               if (footstepPlanToExecute.isEmpty())
               {
                  state.getExecutionState().setValue(FootstepPlanActionExecutionState.PLANNING_FAILED);
               }
               else
               {
                  state.getExecutionState().setValue(FootstepPlanActionExecutionState.PLANNING_SUCCEEDED);
               }
            }
         }
         case PLANNING_FAILED ->
         {
            state.getLogger().error("No planned steps to execute!");
            state.setIsExecuting(false);
            state.setFailed(true);
         }
         case PLANNING_SUCCEEDED ->
         {
            state.setIsExecuting(true);
            buildAndSendCommandAndSetDesiredState();
            state.getExecutionState().setValue(FootstepPlanActionExecutionState.PLAN_COMMANDED);
         }
         case PLAN_COMMANDED ->
         {
            updateProgress();
         }
      }
   }

   private void startFootstepPlanningAsync()
   {
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
                                        footstepPlannerRequest.setStartFootPoses(startFootPosesForThread.get(RobotSide.LEFT),
                                                                                 startFootPosesForThread.get(RobotSide.RIGHT));
                                        // TODO: Set start footholds!!
                                        for (RobotSide side : RobotSide.values)
                                        {
                                           footstepPlannerRequest.setGoalFootPose(side, goalFootPosesForThread.get(side));
                                        }

                                        footstepPlannerRequest.setAssumeFlatGround(true); // TODO: Incorporate height map

                                        footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
                                        double idealFootstepLength = 0.5;
                                        footstepPlanner.getFootstepPlannerParameters().setIdealFootstepLength(idealFootstepLength);
                                        footstepPlanner.getFootstepPlannerParameters().setMaximumStepReach(idealFootstepLength);
                                        state.getLogger().info("Planning footsteps...");
                                        FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
                                        FootstepPlan footstepPlan = footstepPlannerOutput.getFootstepPlan();
                                        state.getLogger()
                                             .info("Footstep planner completed with {}, {} step(s)",
                                                   footstepPlannerOutput.getFootstepPlanningResult(),
                                                   footstepPlan.getNumberOfSteps());

                                        if (footstepPlan.getNumberOfSteps() < 1) // failed
                                        {
                                           FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanner);
                                           rejectionReasonReport.update();
                                           for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
                                           {
                                              double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
                                              state.getLogger()
                                                   .info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
                                           }
                                           state.getLogger().info("Footstep planning failure...");
                                           footstepPlanNotification.set(new FootstepPlan());
                                        }
                                        else
                                        {
                                           for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
                                           {
                                              if (i == 0)
                                                 footstepPlan.getFootstep(i).setTransferDuration(getDefinition().getTransferDuration() / 2.0);
                                              else
                                                 footstepPlan.getFootstep(i).setTransferDuration(getDefinition().getTransferDuration());

                                              footstepPlan.getFootstep(i).setSwingDuration(getDefinition().getSwingDuration());
                                           }
                                           footstepPlanNotification.set(new FootstepPlan(footstepPlan)); // Copy of the output to be safe
                                        }

                                        FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
                                        footstepPlannerLogger.logSession();
                                        FootstepPlannerLogger.deleteOldLogs();
                                     }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   private void buildAndSendCommandAndSetDesiredState()
   {
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlanToExecute,
                                                                                                                    definition.getSwingDuration(),
                                                                                                                    definition.getTransferDuration());
      double finalTransferDuration = 0.01; // We don't want any unecessary pauses at the end; but it can't be 0
      footstepDataListMessage.setFinalTransferDuration(finalTransferDuration);
      footstepDataListMessage.getQueueingProperties().setExecutionMode(definition.getExecutionMode().getValue().toByte());
      footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      state.getLogger().info("Commanding {} footsteps", footstepDataListMessage.getFootstepDataList().size());
      ros2ControllerHelper.publishToController(footstepDataListMessage);
      for (RobotSide side : RobotSide.values)
      {
         trackingCalculators.get(side).reset();
      }

      nominalExecutionDuration = PlannerTools.calculateNominalTotalPlanExecutionDuration(footstepPlanToExecute,
                                                                                         definition.getSwingDuration(),
                                                                                         walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                                         definition.getTransferDuration(),
                                                                                         finalTransferDuration);
      for (RobotSide side : RobotSide.values)
      {
         indexOfLastFoot.put(side, -1);
      }
      for (int i = 0; i < footstepPlanToExecute.getNumberOfSteps(); i++)
      {
         indexOfLastFoot.put(footstepPlanToExecute.getFootstep(i).getRobotSide(), i);
      }

      for (RobotSide side : RobotSide.values)
      {
         int indexOfLastFootSide = indexOfLastFoot.get(side);
         if (indexOfLastFootSide >= 0)
         {
            commandedGoalFeetPoses.get(side).setIncludingFrame(footstepPlanToExecute.getFootstep(indexOfLastFootSide).getFootstepPose());
         }
         else
         {
            commandedGoalFeetPoses.get(side).setIncludingFrame(syncedFeetPoses.get(side));
         }

         state.getDesiredFootPoses().get(side).getValue().clear();
         state.getDesiredFootPoses().get(side).addTrajectoryPoint(syncedFeetPoses.get(side), 0.0);
      }

      for (int i = 0; i < footstepPlanToExecute.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = footstepPlanToExecute.getFootstep(i);
         double stepCompletionTime = PlannerTools.calculateFootstepCompletionTime(footstepPlanToExecute,
                                                                                  definition.getSwingDuration(),
                                                                                  walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                                  definition.getTransferDuration(),
                                                                                  walkingControllerParameters.getDefaultFinalTransferTime(),
                                                                                  i + 1);
         state.getDesiredFootPoses().get(footstep.getRobotSide()).addTrajectoryPoint(footstep.getFootstepPose(), stepCompletionTime);
      }
   }

   private void updateProgress()
   {
      boolean hitTimeLimit = false;
      boolean meetsDesiredCompletionCriteria = true;

      for (RobotSide side : RobotSide.values)
      {
         trackingCalculators.get(side).computeExecutionTimings(nominalExecutionDuration);
         trackingCalculators.get(side).computePoseTrackingData(commandedGoalFeetPoses.get(side), syncedFeetPoses.get(side));
         trackingCalculators.get(side).factorInR3Errors(POSITION_TOLERANCE);
         trackingCalculators.get(side).factoryInSO3Errors(ORIENTATION_TOLERANCE);
         meetsDesiredCompletionCriteria &= trackingCalculators.get(side).isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculators.get(side).getTimeIsUp();
         hitTimeLimit |= trackingCalculators.get(side).getHitTimeLimit();
      }

      int incompleteFootsteps = controllerStatusTracker.getFootstepTracker().getNumberOfIncompleteFootsteps();
      boolean isWalking = controllerStatusTracker.isWalking();
      meetsDesiredCompletionCriteria &= incompleteFootsteps == 0;
      meetsDesiredCompletionCriteria &= !isWalking;

      if (meetsDesiredCompletionCriteria || hitTimeLimit)
      {
         state.setIsExecuting(false);
      }
      if (hitTimeLimit)
      {
         state.setFailed(true);
         state.getLogger().info("Walking failed. (time limit)");
      }
      state.setNominalExecutionDuration(nominalExecutionDuration);
      state.setElapsedExecutionTime(trackingCalculators.get(RobotSide.LEFT).getElapsedTime());
      state.setTotalNumberOfFootsteps(footstepPlanToExecute.getNumberOfSteps());
      state.setNumberOfIncompleteFootsteps(incompleteFootsteps);
      for (RobotSide side : RobotSide.values)
      {
         state.getCurrentFootPoses().get(side).getValue().set(syncedFeetPoses.get(side));
      }
   }
}
