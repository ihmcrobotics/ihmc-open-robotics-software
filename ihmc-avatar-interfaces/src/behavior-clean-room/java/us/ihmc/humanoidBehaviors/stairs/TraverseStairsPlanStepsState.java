package us.ihmc.humanoidBehaviors.stairs;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import org.opencv.xfeatures2d.HarrisLaplaceFeatureDetector;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicReference;

public class TraverseStairsPlanStepsState implements State
{
   private static final String footstepPlannerParameterFileName = "atlasFootstepPlannerParameters_Stairs.ini";

   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final AtomicReference<Pose3D> goalInput = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();

   private final RemoteSyncedRobotModel remoteSyncedRobotModel;
   private final FootstepPlanningModule planningModule;
   private FootstepPlannerOutput output;

   public TraverseStairsPlanStepsState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters)
   {
      this.helper = helper;
      this.parameters = parameters;
      helper.createROS2Callback(TraverseStairsBehaviorAPI.GOAL_INPUT, goalPose ->
      {
         LogTools.debug("Received goal input: " + goalPose);
         goalInput.set(goalPose);
      });
      helper.createROS2Callback(ROS2Tools.LIDAR_REA_REGIONS, planarRegions::set);

      remoteSyncedRobotModel = helper.getOrCreateRobotInterface().newSyncedRobot();
      planningModule = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());
      planningModule.getFootstepPlannerParameters().load(footstepPlannerParameterFileName);
   }

   @Override
   public void onEntry()
   {
      LogTools.debug("Entering " + getClass().getSimpleName());

      if (goalInput.get() == null)
      {
         String message = "No goal received in traverse stairs behavior";
         LogTools.debug(message);
         throw new RuntimeException(message);
      }
      else if (planarRegions.get() == null)
      {
         String message = "No regions received in traverse stairs behavior";
         LogTools.debug(message);
         throw new RuntimeException(message);
      }
   }

   @Override
   public void doAction(double timeInState)
   {
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

      RobotSide steppingSide = planningModule.getFootstepPlannerParameters().getStepOnlyWithRequestedSide();
      request.setRequestedInitialStanceSide(steppingSide.getOppositeSide());

      request.setStartFootPoses(solePoses.get(RobotSide.LEFT), solePoses.get(RobotSide.RIGHT));
      request.setSwingPlannerType(SwingPlannerType.PROPORTION);
      request.setTimeout(parameters.get(TraverseStairsBehaviorParameters.planningTimeout));

      int targetNumberOfFootsteps = 2 * parameters.get(TraverseStairsBehaviorParameters.numberOfStairsPerExecution);
      planningModule.clearCustomTerminationConditions();
      planningModule.addCustomTerminationCondition((plannerTime, iterations, bestPathFinalStep, bestPathSize) -> bestPathSize >= targetNumberOfFootsteps);

      LogTools.debug(getClass().getSimpleName() + ": planning");
      this.output = planningModule.handleRequest(request);
      LogTools.debug(getClass().getSimpleName() + ": " + output.getFootstepPlanningResult());

      // generate log
      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(planningModule);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(50), "FootstepPlanLogDeletion");
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
      return searchWasSuccessful();
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
