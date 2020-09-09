package us.ihmc.humanoidBehaviors.stairs;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.geometry.Side;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.FootstepPlanningResult.FOUND_SOLUTION;

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
      helper.createROS2Callback(TraverseStairsBehaviorAPI.GOAL_INPUT, goalInput::set);
      helper.createROS2Callback(ROS2Tools.LIDAR_REA_REGIONS, planarRegions::set);

      remoteSyncedRobotModel = helper.getOrCreateRobotInterface().newSyncedRobot();
      planningModule = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());
      planningModule.getFootstepPlannerParameters().load(footstepPlannerParameterFileName);
   }

   @Override
   public void onEntry()
   {
      if (goalInput.get() == null)
      {
         throw new RuntimeException("No goal received in traverse stairs behavior");
      }
      else if (planarRegions.get() == null)
      {
         throw new RuntimeException("No regions received in traverse stairs behavior");
      }

      int targetNumberOfFootsteps = 2 * parameters.get(TraverseStairsBehaviorParameters.numberOfStairsPerExecution);
      planningModule.clearCustomTerminationConditions();
      planningModule.addCustomTerminationCondition((time, iterations, finalStep, pathSize) -> (pathSize % 2 == 0) && (pathSize >= targetNumberOfFootsteps));
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
      }

      request.setStartFootPoses(solePoses.get(RobotSide.LEFT), solePoses.get(RobotSide.RIGHT));
      request.setSwingPlannerType(SwingPlannerType.PROPORTION);
      request.setTimeout(12.0);

      this.output = planningModule.handleRequest(request);

      // generate log
      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(planningModule);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(50), "FootstepPlanLogDeletion");
   }

   boolean shouldTransitionToExecute(double timeInState)
   {
      return output.getFootstepPlanningResult() == FOUND_SOLUTION;
   }

   boolean shouldTransitionBackToPause(double timeInState)
   {
      return output.getFootstepPlanningResult() != FOUND_SOLUTION;
   }

   FootstepPlannerOutput getOutput()
   {
      return output;
   }
}
