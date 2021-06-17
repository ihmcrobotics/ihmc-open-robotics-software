package us.ihmc.behaviors.stairs;

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
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI.*;

public class TraverseStairsPlanStepsState implements State
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final AtomicBoolean operatorReviewEnabled;
   private final AtomicReference<Pose3D> goalInput = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepListPublisher;

   private final ROS2SyncedRobotModel ROS2SyncedRobotModel;
   private final FootstepPlanningModule planningModule;
   private FootstepPlannerOutput output;

   private final AtomicBoolean executeStepsSignaled = new AtomicBoolean();
   private final AtomicBoolean planSteps = new AtomicBoolean();
   private boolean isStillPlanning = false;

   public TraverseStairsPlanStepsState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters, AtomicBoolean operatorReviewEnabled)
   {
      this.helper = helper;
      this.parameters = parameters;
      this.operatorReviewEnabled = operatorReviewEnabled;
      helper.subscribeViaCallback(TraverseStairsBehaviorAPI.GOAL_INPUT, goalPose ->
      {
         LogTools.info("Received goal input: " + goalPose);
         goalInput.set(goalPose);
      });
      helper.subscribeViaCallback(ROS2Tools.LIDAR_REA_REGIONS, planarRegions::set);

      ROS2SyncedRobotModel = helper.getOrCreateRobotInterface().newSyncedRobot();
      planningModule = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());
      FootstepPlannerParametersBasics footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters("_Stairs");
      planningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);
      SwingPlannerParametersBasics swingPlannerParameters = helper.getRobotModel().getSwingPlannerParameters("_Stairs");
      planningModule.getSwingPlannerParameters().set(swingPlannerParameters);
      helper.subscribeViaCallback(FootstepPlannerParameters, parametersAsStrings ->
      {
         helper.getOrCreateStatusLogger().info("Accepting new footstep planner parameters");
         planningModule.getFootstepPlannerParameters().setAllFromStrings(parametersAsStrings);
      });
      helper.subscribeViaCallback(SwingPlannerParameters, parametersAsStrings ->
      {
         helper.getOrCreateStatusLogger().info("Accepting new swing planner parameters");
         planningModule.getSwingPlannerParameters().setAllFromStrings(parametersAsStrings);
      });

      footstepListPublisher = new IHMCROS2Publisher<>(helper.getROS2Node(), TraverseStairsBehaviorAPI.PLANNED_STEPS);
      new IHMCROS2Callback<>(helper.getROS2Node(), TraverseStairsBehaviorAPI.EXECUTE_STEPS, r -> executeStepsSignaled.set(true));
      new IHMCROS2Callback<>(helper.getROS2Node(), TraverseStairsBehaviorAPI.REPLAN, r -> planSteps.set(true));
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
//         throw new RuntimeException(message);
      }
      else if (planarRegions.get() == null)
      {
         String message = "No regions received in traverse stairs behavior";
         LogTools.info(message);
//         throw new RuntimeException(message);
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
      isStillPlanning = true;
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

      ROS2SyncedRobotModel.update();
      SideDependentList<Pose3D> solePoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         MovingReferenceFrame soleFrame = ROS2SyncedRobotModel.getReferenceFrames().getSoleFrame(robotSide);
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
      isStillPlanning = false;
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

   public boolean searchWasSuccessful()
   {
      if (output != null && output.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION)
      {
         return true;
      }
      else if (output != null && output.getFootstepPlanningResult() == FootstepPlanningResult.HALTED)
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
      return searchWasSuccessful() && (!operatorReviewEnabled.get() || executeStepsSignaled.get());
   }

   boolean shouldTransitionBackToPause(double timeInState)
   {
      return !searchWasSuccessful();
   }

   FootstepPlannerOutput getOutput()
   {
      return output;
   }

   public boolean isStillPlanning()
   {
      return isStillPlanning;
   }
}
