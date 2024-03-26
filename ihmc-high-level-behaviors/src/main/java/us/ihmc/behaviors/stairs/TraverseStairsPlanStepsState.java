package us.ihmc.behaviors.stairs;

import controller_msgs.msg.dds.FootstepDataListMessage;
import toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
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
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.TimerSnapshot;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI.*;

public class TraverseStairsPlanStepsState extends TraverseStairsState
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final AtomicBoolean operatorReviewEnabled;
   private final AtomicReference<Pose3D> goalInput = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepListPublisher;

   private final ROS2SyncedRobotModel syncedRobot;
   private final FootstepPlanningModule planningModule;
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("RegionsRelay", true);
   private FootstepPlannerOutput output;

   private final AtomicBoolean executeStepsSignaled = new AtomicBoolean();
   private final AtomicBoolean planSteps = new AtomicBoolean();
   private boolean isStillPlanning = false;
   private final StatusLogger statusLogger;

   public TraverseStairsPlanStepsState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters, AtomicBoolean operatorReviewEnabled)
   {
      this.helper = helper;
      this.parameters = parameters;
      this.operatorReviewEnabled = operatorReviewEnabled;
      this.statusLogger = helper.getOrCreateStatusLogger();

      helper.subscribeViaCallback(TraverseStairsBehaviorAPI.GOAL_INPUT, goalPose ->
      {
         statusLogger.info("Received goal input: " + goalPose);
         goalInput.set(goalPose);
      });
      helper.subscribeViaCallback(PerceptionAPI.LIDAR_REA_REGIONS, newValue ->
      {
         planarRegions.set(newValue);
         executor.submit(() ->
         {
//            helper.publish(PlanarRegionsForUI, PlanarRegionMessageConverter.convertToPlanarRegionsList(newValue));
         });
      });

      syncedRobot = helper.getOrCreateRobotInterface().newSyncedRobot();
      planningModule = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());
      FootstepPlannerParametersBasics footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters("_Stairs");
      planningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);
      SwingPlannerParametersBasics swingPlannerParameters = helper.getRobotModel().getSwingPlannerParameters("_Stairs");
      planningModule.getSwingPlannerParameters().set(swingPlannerParameters);
//      helper.subscribeViaCallback(FootstepPlannerParameters, parametersAsStrings ->
//      {
//         statusLogger.info("Accepting new footstep planner parameters");
//         planningModule.getFootstepPlannerParameters().setAllFromStrings(parametersAsStrings);
//      });
//      helper.subscribeViaCallback(SwingPlannerParameters, parametersAsStrings ->
//      {
//         statusLogger.info("Accepting new swing planner parameters");
//         planningModule.getSwingPlannerParameters().setAllFromStrings(parametersAsStrings);
//      });

      footstepListPublisher = new IHMCROS2Publisher<>(helper.getROS2Node(), TraverseStairsBehaviorAPI.PLANNED_STEPS);
      new ROS2Callback<>(helper.getROS2Node(), TraverseStairsBehaviorAPI.EXECUTE_STEPS, r -> executeStepsSignaled.set(true));
      new ROS2Callback<>(helper.getROS2Node(), TraverseStairsBehaviorAPI.REPLAN, r -> planSteps.set(true));
   }

   @Override
   public void onEntry()
   {
      statusLogger.info("Entering " + getClass().getSimpleName());
      reset();

      if (goalInput.get() == null)
      {
         String message = "No goal received in traverse stairs behavior";
         statusLogger.info(message);
      }
      else if (planarRegions.get() == null)
      {
         String message = "No regions received in traverse stairs behavior";
         statusLogger.info(message);
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

   @Override
   public void onExit(double timeInState)
   {
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
         statusLogger.info("Waiting for previous plan to complete...");
         ThreadTools.sleep(10);
      }

      TimerSnapshot timerSnapshot = syncedRobot.getDataReceptionTimerSnapshot();
      boolean hasReceivedRecentData = timerSnapshot.isRunning(0.5);
      if (!hasReceivedRecentData)
      {
         statusLogger.info("Has not received robot configuration data recently");
         return;
      }

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegions.get()))));
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalInput.get());
      request.setPlanBodyPath(false);

      syncedRobot.update();
      SideDependentList<Pose3D> solePoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         MovingReferenceFrame soleFrame = syncedRobot.getReferenceFrames().getSoleFrame(robotSide);
         FramePose3D solePose = new FramePose3D(soleFrame);
         solePose.changeFrame(ReferenceFrame.getWorldFrame());
         solePoses.put(robotSide, new Pose3D(solePose));
      }

      request.setStartFootPoses(solePoses.get(RobotSide.LEFT), solePoses.get(RobotSide.RIGHT));
      request.setSwingPlannerType(SwingPlannerType.PROPORTION);
      request.setTimeout(parameters.get(TraverseStairsBehaviorParameters.planningTimeout));

      int targetNumberOfFootsteps = 2 * parameters.get(TraverseStairsBehaviorParameters.numberOfStairsPerExecution);
      planningModule.clearCustomTerminationConditions();

      planningModule.addCustomTerminationCondition((plannerTime, iterations, bestPathFinalStep, bestSecondToFinalState, bestPathSize) -> bestPathSize >= targetNumberOfFootsteps);

      statusLogger.info(getClass().getSimpleName() + ": planning");
      this.output = planningModule.handleRequest(request);
      statusLogger.info(getClass().getSimpleName() + " number of steps in plan: " + output.getFootstepPlan().getNumberOfSteps());

      // remove extra steps
      while (output.getFootstepPlan().getNumberOfSteps() > targetNumberOfFootsteps)
      {
         output.getFootstepPlan().remove(output.getFootstepPlan().getNumberOfSteps() - 1);
      }

      // remove final step if on different stair
      int numberOfSteps = output.getFootstepPlan().getNumberOfSteps();
      if (numberOfSteps >= 2)
      {
         double onSameStairThreshold = 0.05;
         double lastStepHeight = output.getFootstepPlan().getFootstep(numberOfSteps - 1).getFootstepPose().getZ();
         double secondStepLastHeight = output.getFootstepPlan().getFootstep(numberOfSteps - 2).getFootstepPose().getZ();
         boolean lastStepsAreOnSameStair = Math.abs(lastStepHeight - secondStepLastHeight) < onSameStairThreshold;
         if (!lastStepsAreOnSameStair)
         {
            output.getFootstepPlan().remove(numberOfSteps - 1);
         }
      }

      // lower height of first step down
      mutateFirstStepDownHeight(solePoses.get(request.getRequestedInitialStanceSide()));

      // generate log
      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(planningModule);
      footstepPlannerLogger.logSession();

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
      if (output == null)
      {
         return false;
      }

      return output.getFootstepPlan().getNumberOfSteps() >= 2;
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

   public void destroy()
   {
      executor.destroy();
   }
}
