package us.ihmc.behaviors.activeMapping.ContinuousHikingStates;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousHikingState;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepPoseHeuristicChecker;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask.statistics;

public class PlanAvailableState implements State
{
   private final HumanoidReferenceFrames referenceFrames;
   private final YoEnum<ContinuousHikingState> continuousHikingState;
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousPlanner continuousPlanner;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final TerrainMapData terrainMap;
   private final TerrainPlanningDebugger debugger;

   private final ROS2Topic controllerFootstepDataTopic;
   private final ROS2PublisherMap publisherMap;
   private final FootstepPoseHeuristicChecker stepChecker;
   private List<QueuedFootstepStatusMessage> controllerQueue;

   private int controllerQueueSize = 0;
   private String message = "";

   public PlanAvailableState(DRCRobotModel robotModel,
                             ROS2Helper ros2Helper,
                             ROS2Node ros2Node,
                             String simpleRobotName,
                             YoRegistry parentRegistry,
                             HumanoidReferenceFrames referenceFrames,
                             YoEnum<ContinuousHikingState> continuousHikingState,
                             AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                             ContinuousPlanner continuousPlanner,
                             ContinuousHikingParameters continuousHikingParameters,
                             TerrainMapData terrainMap,
                             TerrainPlanningDebugger debugger)
   {
      this.referenceFrames = referenceFrames;
      this.continuousHikingState = continuousHikingState;
      this.commandMessage = commandMessage;
      this.continuousPlanner = continuousPlanner;
      this.continuousHikingParameters = continuousHikingParameters;
      this.terrainMap = terrainMap;
      this.debugger = debugger;

      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);

      controllerFootstepDataTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, simpleRobotName);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      // FIXME this needs to get a copy of the height map or terrain map for the step checker to actually work as intended.
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(FootstepPlanningModuleLauncher.createFootPolygons(robotModel),
                                                                  continuousPlanner.getFootstepPlannerParameters(),
                                                                  environmentHandler);
      stepChecker = new FootstepPoseHeuristicChecker(continuousPlanner.getFootstepPlannerParameters(), snapper, parentRegistry);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      // Set the that controller queue size before getting the new one
      statistics.setLastFootstepQueueLength(controllerQueueSize);

      if (!continuousHikingParameters.getEnableContinuousWalking())
         return;

      controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
      if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
      {
         LogTools.warn(message = String.format("State: [%s]: Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size(),
                                               continuousHikingState.getEnumValue()));
         statistics.appendString(message);
      }
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   @Override
   public void onEntry()
   {
      LogTools.warn(continuousHikingState.getEnumValue());
   }

   @Override
   public void doAction(double timeInState)
   {
      FootstepDataListMessage footstepDataList = continuousPlanner.getLimitedFootstepDataListMessage(continuousHikingParameters, controllerQueue);

      debugger.publishPlannedFootsteps(footstepDataList);
      if (commandMessage.get().getUseHybridPlanner() || commandMessage.get().getUseMonteCarloFootstepPlanner() || commandMessage.get()
                                                                                                                                .getUseMonteCarloPlanAsReference())
      {
         debugger.publishMonteCarloPlan(continuousPlanner.getMonteCarloFootstepDataListMessage());
         debugger.publishMonteCarloNodesForVisualization(continuousPlanner.getMonteCarloFootstepPlanner().getRoot(), terrainMap);
      }

      if (continuousHikingParameters.getStepPublisherEnabled())
      {
         LogTools.info(message = String.format("State: [%s]: Sending (" + footstepDataList.getFootstepDataList().size() + ") steps to controller",
                                               continuousHikingState.getEnumValue()));
         statistics.appendString(message);

         FramePose3DReadOnly stanceFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(), continuousPlanner.getImminentFootstepPose());

         FramePose3DReadOnly candidateStepPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                 footstepDataList.getFootstepDataList().get(0).getLocation(),
                                                                 footstepDataList.getFootstepDataList().get(0).getOrientation());

         BipedalFootstepPlannerNodeRejectionReason reason;
         FramePose3D startOfSwingPose = new FramePose3D();

         RobotSide stepSide = continuousPlanner.getImminentFootstepSide();
         assert stepSide != null;
         startOfSwingPose.setFromReferenceFrame(referenceFrames.getSoleFrame(stepSide.getOppositeSide()));
         reason = stepChecker.checkValidity(stepSide.getOppositeSide(), candidateStepPose, stanceFootPose, startOfSwingPose);

         if (reason == null)
         {
            publisherMap.publish(controllerFootstepDataTopic, footstepDataList);

            continuousHikingState.set(ContinuousHikingState.WAITING_TO_LAND);
            continuousPlanner.setPlanAvailable(false);
            continuousPlanner.transitionCallback();
            statistics.setStartWaitingTime();
         }
         else
         {
            continuousHikingParameters.setEnableContinuousWalking(false);
            LogTools.error("Planning failed:" + reason.name());
            LogTools.info("Start of Swing:" + startOfSwingPose);
            LogTools.info("Stance:" + stanceFootPose);
            LogTools.info("Candidate:" + candidateStepPose);

            continuousPlanner.setInitialized(false);
            continuousHikingState.set(ContinuousHikingState.NOT_STARTED);
         }
      }
      else
      {
         continuousHikingState.set(ContinuousHikingState.READY_TO_PLAN);
      }
   }

   @Override
   public void onExit(double timeInState)
   {
      if (!continuousHikingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
         continuousHikingState.set(ContinuousHikingState.NOT_STARTED);
   }
}
