package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import org.apache.commons.lang.time.StopWatch;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.behaviors.activeMapping.ContinuousPlannerStatistics;
import us.ihmc.behaviors.activeMapping.ContinuousPlanningTools;
import us.ihmc.behaviors.activeMapping.ControllerFootstepQueueMonitor;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class ReadyToPlanState implements State
{
   private final HumanoidReferenceFrames referenceFrames;
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousPlanner continuousPlanner;
   private final ControllerFootstepQueueMonitor controllerFootstepQueueMonitor;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final TerrainMapData terrainMap;
   private final TerrainPlanningDebugger debugger;
   private final ContinuousPlannerStatistics statistics;
   private final List<SideDependentList<Pose3D>> walkToGoalWayPointList = new ArrayList<>();
   private final Point3D robotLocation = new Point3D();
   private final StopWatch stopWatch = new StopWatch();
   double timeInSwingToStopPlanningAndWaitTillNextAttempt = 0;
   private PlanningMode mode;
   public ReadyToPlanState(ROS2Helper ros2Helper,
                           HumanoidReferenceFrames referenceFrames,
                           AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                           ContinuousPlanner continuousPlanner,
                           ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                           ContinuousHikingParameters continuousHikingParameters,
                           TerrainMapData terrainMap,
                           TerrainPlanningDebugger debugger,
                           ContinuousPlannerStatistics statistics)
   {
      this.referenceFrames = referenceFrames;
      this.commandMessage = commandMessage;
      this.continuousPlanner = continuousPlanner;
      this.controllerFootstepQueueMonitor = controllerFootstepQueueMonitor;
      this.continuousHikingParameters = continuousHikingParameters;
      this.terrainMap = terrainMap;
      this.debugger = debugger;
      this.statistics = statistics;

      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.PLACED_GOAL_FOOTSTEPS, this::addWayPointCheckPointToList);
   }

   @Override
   public void onEntry()
   {
      continuousPlanner.setPlanAvailable(false);
      stopWatch.reset();
      debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
      timeInSwingToStopPlanningAndWaitTillNextAttempt = continuousHikingParameters.getSwingTime() * continuousHikingParameters.getPercentThroughSwingToPlanTo();
      stopWatch.start();
   }

   @Override
   public void doAction(double timeInState)
   {
      statistics.setLastAndTotalWaitingTimes();

      // These may be null if no steps have been sent to the controller, good to check that here
      if (controllerFootstepQueueMonitor.getFootstepStatusMessage() != null && controllerFootstepQueueMonitor.getControllerFootstepQueue() != null)
      {
         continuousPlanner.setLatestFootstepStatusMessage(controllerFootstepQueueMonitor.getFootstepStatusMessage());
         continuousPlanner.setLatestControllerQueue(controllerFootstepQueueMonitor.getControllerFootstepQueue());
      }

      // Set up the imminent stance and goal poses in which to plan from
      continuousPlanner.setImminentStanceToPlanFrom();
      SideDependentList<FramePose3D> goalPoses = returnGoalStuff();
      continuousPlanner.setGoalWaypointPoses(goalPoses.get(RobotSide.LEFT), goalPoses.get(RobotSide.RIGHT));
      debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());

      // Plan to the goal and log the plan
      continuousPlanner.planToGoal(commandMessage.get());
      continuousPlanner.logFootStePlan();

      if (commandMessage.get().getUseHybridPlanner() || commandMessage.get().getUseMonteCarloFootstepPlanner() || commandMessage.get()
                                                                                                                                .getUseMonteCarloPlanAsReference())
      {
         debugger.publishMonteCarloPlan(continuousPlanner.getMonteCarloFootstepDataListMessage());
         debugger.publishMonteCarloNodesForVisualization(continuousPlanner.getMonteCarloFootstepPlanner().getRoot(), terrainMap);
      }

      // We know that we have a plan, and that only gets set to true when we have at least one step in the plan, so we know it's not empty
      if (continuousPlanner.isPlanAvailable())
      {
         FootstepDataListMessage message = FootstepDataMessageConverter.createFootstepDataListFromPlan(continuousPlanner.getLatestFootstepPlan(),
                                                                                                       continuousHikingParameters.getSwingTime(),
                                                                                                       continuousHikingParameters.getTransferTime());
         debugger.publishPlannedFootsteps(message);
      }
   }

   public void addWayPointToList(Pose3D leftFootGoalPose, Pose3D rightFootGoalPose)
   {
      mode = PlanningMode.WALK_TO_GOAL;
      //TODO make sure we don't add the same values twice
      SideDependentList<Pose3D> latestWayPoint = new SideDependentList<>();
      latestWayPoint.put(RobotSide.LEFT, leftFootGoalPose);
      latestWayPoint.put(RobotSide.RIGHT, rightFootGoalPose);

      LogTools.info("Added waypoint for WALK_TO_GOAL");
      walkToGoalWayPointList.add(latestWayPoint);
   }

   public void addWayPointCheckPointToList(PoseListMessage poseListMessage)
   {
      List<Pose3D> poses = MessageTools.unpackPoseListMessage(poseListMessage);
      addWayPointToList(poses.get(0), poses.get(1));
      debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
   }

   float xRandomMargin = 0.2f;
   float nominalStanceWidth = 0.22f;

   public SideDependentList<FramePose3D> returnGoalStuff()
   {
      SideDependentList<FramePose3D> goalPoses = new SideDependentList<>();

      if (walkToGoalWayPointList.isEmpty())
      {
         ContinuousPlanningTools.setRandomizedStraightGoalPoses(continuousPlanner.getWalkingStartMidPose(),
                                                                continuousPlanner.getStartingStancePose(),
                                                                goalPoses,
                                                                (float) continuousHikingParameters.getGoalPoseForwardDistance(),
                                                                xRandomMargin,
                                                                (float) continuousHikingParameters.getGoalPoseUpDistance(),
                                                                nominalStanceWidth);

         return goalPoses;
      }

      Vector3DBasics robotLocationVector = referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame().getTranslation();
      robotLocation.set(robotLocationVector);
      double distanceToGoalPose = ContinuousPlanningTools.getDistanceFromRobotToGoalPoseOnXYPlane(robotLocation, goalPoses);

      if (distanceToGoalPose < continuousHikingParameters.getNextWaypointDistanceMargin())
      {
         LogTools.info("Removed goal from list... ready to go to the next one");
         walkToGoalWayPointList.remove(0);

         if (!walkToGoalWayPointList.isEmpty())
         {
            goalPoses.get(RobotSide.LEFT).set(walkToGoalWayPointList.get(0).get(RobotSide.LEFT));
            goalPoses.get(RobotSide.RIGHT).set(walkToGoalWayPointList.get(0).get(RobotSide.RIGHT));
         }
         else
         {
            continuousHikingParameters.setEnableContinuousWalking(false);
         }
      }

      return goalPoses;
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      boolean stopPlanningAndCompleteCurrentStep = stopWatch.getTime() > timeInSwingToStopPlanningAndWaitTillNextAttempt;
      return stopPlanningAndCompleteCurrentStep || (continuousPlanner.isPlanAvailable() && continuousHikingParameters.getStepPublisherEnabled());
   }

   public enum PlanningMode
   {
      EXECUTE_AND_PAUSE, FRONTIER_EXPANSION, ACTIVE_SEARCH, FAST_HIKING, WALK_TO_GOAL
   }
}
