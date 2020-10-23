package us.ihmc.humanoidBehaviors.stairs;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class TraverseStairsPlanStepsState implements State
{
   private static final String footstepPlannerParameterFileName = "atlasFootstepPlannerParameters_Stairs.ini";
   private static final String swingParameterFileName = "atlasSwingPlannerParameters_Stairs.ini";

   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final AtomicReference<Pose3D> goalInput = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepListPublisher;

   private final RemoteSyncedRobotModel remoteSyncedRobotModel;
   private final FootstepPlanningModule planningModule;
   private FootstepPlannerOutput output;

   private final AtomicBoolean executeStepsSignaled = new AtomicBoolean();
   private final AtomicBoolean planSteps = new AtomicBoolean();

   public TraverseStairsPlanStepsState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters)
   {
      this.helper = helper;
      this.parameters = parameters;
      helper.createROS2Callback(TraverseStairsBehaviorAPI.GOAL_INPUT, goalPose ->
      {
         LogTools.info("Received goal input: " + goalPose);
         goalInput.set(goalPose);
      });
      helper.createROS2Callback(ROS2Tools.LIDAR_REA_REGIONS, planarRegions::set);

      remoteSyncedRobotModel = helper.getOrCreateRobotInterface().newSyncedRobot();
      planningModule = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());
      planningModule.getFootstepPlannerParameters().load(footstepPlannerParameterFileName);
      planningModule.getSwingPlannerParameters().load(swingParameterFileName);

      footstepListPublisher = new IHMCROS2Publisher<>(helper.getManagedROS2Node(), TraverseStairsBehaviorAPI.PLANNED_STEPS);
      new IHMCROS2Callback<>(helper.getManagedROS2Node(), TraverseStairsBehaviorAPI.EXECUTE_STEPS, r -> executeStepsSignaled.set(true));
      new IHMCROS2Callback<>(helper.getManagedROS2Node(), TraverseStairsBehaviorAPI.REPLAN, r -> planSteps.set(true));
   }

   @Override
   public void onEntry()
   {
      LogTools.info("Entering " + getClass().getSimpleName());
      reset();

      if (goalInput.get() == null)
      {
         String message = "No goal received in traverse stairs behavior";
         LogTools.info(message);
         throw new RuntimeException(message);
      }
      else if (planarRegions.get() == null)
      {
         String message = "No regions received in traverse stairs behavior";
         LogTools.info(message);
         throw new RuntimeException(message);
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      if (planSteps.getAndSet(false))
      {
         planSteps();
      }
   }

   public void reset()
   {
      output = null;
      executeStepsSignaled.set(false);
      planSteps.set(true);
   }

   private void planSteps()
   {
      if (planningModule.isPlanning())
      {
         planningModule.halt();
      }

      while (planningModule.isPlanning())
      {
         // wait...
         LogTools.info("Waiting for previous plan to complete...");
         ThreadTools.sleep(10);
      }

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setPlanarRegionsList(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegions.get()));
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalInput.get());
      request.setPlanBodyPath(false);

      remoteSyncedRobotModel.update();
      SideDependentList<Pose3D> solePoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         MovingReferenceFrame soleFrame = remoteSyncedRobotModel.getReferenceFrames().getSoleFrame(robotSide);
         FramePose3D solePose = new FramePose3D(soleFrame);
         solePose.changeFrame(ReferenceFrame.getWorldFrame());
         solePoses.put(robotSide, new Pose3D(solePose));
      }

      request.setStartFootPoses(solePoses.get(RobotSide.LEFT), solePoses.get(RobotSide.RIGHT));
      request.setSwingPlannerType(SwingPlannerType.PROPORTION);
      request.setTimeout(parameters.get(TraverseStairsBehaviorParameters.planningTimeout));

      int targetNumberOfFootsteps = 2 * parameters.get(TraverseStairsBehaviorParameters.numberOfStairsPerExecution);
      planningModule.clearCustomTerminationConditions();

      double onSameStairThreshold = 0.05;
      planningModule.addCustomTerminationCondition((plannerTime, iterations, bestPathFinalStep, bestSecondToFinalState, bestPathSize) ->
                                                   {
                                                      boolean longEnoughPath = bestPathSize >= targetNumberOfFootsteps;
                                                      boolean finalStepsOnSameStair = Math.abs(bestPathFinalStep.getTranslationZ() - bestSecondToFinalState.getTranslationZ()) < onSameStairThreshold;
                                                      return longEnoughPath && finalStepsOnSameStair;
                                                   });

      LogTools.info(getClass().getSimpleName() + ": planning");
      this.output = planningModule.handleRequest(request);
      LogTools.info(getClass().getSimpleName() + " numer of steps in plan: " + output.getFootstepPlan().getNumberOfSteps());

      // lower height of first step down
      mutateFirstStepDownHeight(solePoses.get(request.getRequestedInitialStanceSide()));

      // generate log
      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(planningModule);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(50), "FootstepPlanLogDeletion");

      // publish to ui
      FootstepPlanningToolboxOutputStatus outputStatus = new FootstepPlanningToolboxOutputStatus();
      planningModule.getOutput().setPacket(outputStatus);
      footstepListPublisher.publish(outputStatus.getFootstepDataList());
   }

   private void mutateFirstStepDownHeight(Pose3DReadOnly initialStancePose)
   {
      double initialStanceHeight = initialStancePose.getZ();

      FootstepPlan footstepPlan = output.getFootstepPlan();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         double previousStepHeight = (i == 0) ? initialStanceHeight : footstepPlan.getFootstep(i - 1).getFootstepPose().getZ();
         double stepHeight = footstepPlan.getFootstep(i).getFootstepPose().getZ();

         if (stepHeight < previousStepHeight - parameters.get(TraverseStairsBehaviorParameters.heightToConsiderStepDown))
         {
            double heightAdjustment = parameters.get(TraverseStairsBehaviorParameters.amountToLowerFirstStepDown);
            footstepPlan.getFootstep(i).getFootstepPose().getPosition().subZ(heightAdjustment);
            break;
         }
      }
   }

   private boolean searchWasSuccessful()
   {
      if (output.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION)
      {
         return true;
      }
      else if (output.getFootstepPlanningResult() == FootstepPlanningResult.HALTED)
      {
         int targetNumberOfFootsteps = 2 * parameters.get(TraverseStairsBehaviorParameters.numberOfStairsPerExecution);
         return output.getFootstepPlan().getNumberOfSteps() >= targetNumberOfFootsteps;
      }
      else
      {
         return false;
      }
   }

   boolean shouldTransitionToExecute(double timeInState)
   {
      return searchWasSuccessful() && executeStepsSignaled.get();
   }

   boolean shouldTransitionBackToPause(double timeInState)
   {
      return !searchWasSuccessful();
   }

   FootstepPlannerOutput getOutput()
   {
      return output;
   }
}
