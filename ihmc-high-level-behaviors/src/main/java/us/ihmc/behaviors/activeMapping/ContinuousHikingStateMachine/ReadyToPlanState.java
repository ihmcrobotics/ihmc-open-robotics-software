package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousHikingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import org.apache.commons.lang3.time.StopWatch;
import us.ihmc.behaviors.activeMapping.ContinuousHikingLogger;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.behaviors.activeMapping.ContinuousPlannerTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.behaviors.activeMapping.TerrainPlanningDebugger;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class ReadyToPlanState implements State
{
   // These could be put into tunable parameters but for now they were left here
   private static final float X_RANDOM_MARGIN = 0.2f;
   private static final float NOMINAL_STANCE_WIDTH = 0.22f;
   private static final double ALPHA = 0.1;
   public static final double JOYSTICK_ALPHA = 0.25;

   private final HumanoidReferenceFrames referenceFrames;
   private final AtomicReference<ContinuousHikingCommandMessage> commandMessage;
   private final ContinuousPlanner continuousPlanner;
   private final ControllerFootstepQueueMonitor controllerFootstepQueueMonitor;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final Supplier<TerrainMapData> terrainSupplier;
   private final TerrainPlanningDebugger debugger;
   private final ContinuousHikingLogger continuousHikingLogger;
   private final List<SideDependentList<FramePose3D>> walkToGoalWayPointPoses = new ArrayList<>();
   private final Point3D robotLocation = new Point3D();
   private final StopWatch stopWatch = new StopWatch();
   double timeInSwingToStopPlanningAndWaitTillNextAttempt = 0;
   private double previousLateralValue;

   /**
    * This state exists to plan footsteps based on the conditions of the {@link ContinuousHikingParameters}. This state publishes visuals the UI but doesn't
    * send anything to the controller.
    * We allow for staying in this state and re-planning over and over without sending any steps. However, if we are sending steps, we only stay in this state
    * for a percentage of the swing, and if we don't get a good plan by then, we will have to try again when the current step is completed, and we get back to
    * this state.
    */
   public ReadyToPlanState(ROS2Helper ros2Helper,
                           HumanoidReferenceFrames referenceFrames,
                           AtomicReference<ContinuousHikingCommandMessage> commandMessage,
                           ContinuousPlanner continuousPlanner,
                           ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                           ContinuousHikingParameters continuousHikingParameters,
                           Supplier<TerrainMapData> terrainSupplier,
                           TerrainPlanningDebugger debugger,
                           ContinuousHikingLogger continuousHikingLogger)
   {
      this.referenceFrames = referenceFrames;
      this.commandMessage = commandMessage;
      this.continuousPlanner = continuousPlanner;
      this.controllerFootstepQueueMonitor = controllerFootstepQueueMonitor;
      this.continuousHikingParameters = continuousHikingParameters;
      this.terrainSupplier = terrainSupplier;
      this.debugger = debugger;
      this.continuousHikingLogger = continuousHikingLogger;

      ros2Helper.subscribeViaCallback(ContinuousHikingAPI.PLACED_GOAL_FOOTSTEPS, this::addWayPointPoseToList);
      ros2Helper.subscribeViaCallback(ContinuousHikingAPI.CLEAR_GOAL_FOOTSTEPS, this::clearWayPointList);
   }

   @Override
   public void onEntry()
   {
      if (controllerFootstepQueueMonitor.getControllerFootstepQueue().isEmpty() && continuousHikingParameters.getStepPublisherEnabled())
      {
         previousLateralValue = 0;
      }

      stopWatch.reset();
      continuousPlanner.setPlanAvailable(false);
      timeInSwingToStopPlanningAndWaitTillNextAttempt = continuousHikingParameters.getSwingTime() * (1 - ALPHA);
      stopWatch.start();
   }

   @Override
   public void doAction(double timeInState)
   {
      // These may be null if no steps have been sent to the controller, good to check that here
      if (controllerFootstepQueueMonitor.getFootstepStatusMessage() != null && controllerFootstepQueueMonitor.getControllerFootstepQueue() != null)
      {
         continuousPlanner.setLatestFootstepStatusMessage(controllerFootstepQueueMonitor.getFootstepStatusMessage());
         continuousPlanner.setLatestControllerQueue(controllerFootstepQueueMonitor.getControllerFootstepQueue());
      }

      double timeInSwingToWait = Conversions.secondsToMilliseconds(continuousHikingParameters.getSwingTime() * continuousHikingParameters.getPercentThroughSwingToStartPlanning());
      if (continuousHikingParameters.getStepPublisherEnabled() && !controllerFootstepQueueMonitor.getControllerFootstepQueue().isEmpty())
      {
         // We attempt to wait based on the parameters. Wait a little less because its better to go a little early then a little late
         LogTools.info("Waiting for " + timeInSwingToWait + " ms!");
         long timeToWait = (long) ((long) (timeInSwingToWait - stopWatch.getTime()) * (1 - ALPHA));
         ThreadTools.sleep(timeToWait);
         LogTools.info("I've waited long enough");
      }

      // Set up the imminent stance and goal poses in which to plan from
      continuousPlanner.setImminentStanceToPlanFrom();
      SideDependentList<FramePose3D> goalPoses = getGoalPosesBasedOnPlanningMode();
      continuousHikingLogger.appendString("Goal Poses: \n" + goalPoses.toString());
      debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartStancePose(), goalPoses);

      // Based on the success of placing a goal, we may have asked the state machine to stop walking if we can't walk to the goal anymore
      if (!commandMessage.get().getEnableContinuousHiking())
      {
         return;
      }

      // Get the latest data from the perception pipeline to be used with the current footstep plan
      TerrainMapData terrainMapData = terrainSupplier.get();

      // Plan to the goal and log the plan
      continuousPlanner.planToGoal(goalPoses, terrainMapData);

      if (continuousHikingParameters.getStepPublisherEnabled())
      {
         continuousPlanner.logFootStePlan();
      }

      if (commandMessage.get().getUseMonteCarloFootstepPlanner() || commandMessage.get().getUseMonteCarloPlanAsReference())
      {
         debugger.publishMonteCarloPlan(continuousPlanner.getMonteCarloFootstepDataListMessage());
         debugger.publishMonteCarloNodesForVisualization(continuousPlanner.getMonteCarloFootstepPlanner().getRoot(), terrainSupplier.get());
      }

      // We know that we have a plan, and this method only gets set to true when we have at least one step in the plan, so we know it's not empty, let's send it
      if (continuousPlanner.isPlanAvailable())
      {
         FootstepDataListMessage message = FootstepDataMessageConverter.createFootstepDataListFromPlan(continuousPlanner.getLatestFootstepPlan(),
                                                                                                       continuousHikingParameters.getSwingTime(),
                                                                                                       continuousHikingParameters.getTransferTime());

         continuousHikingLogger.appendString("FootstepDataListMessage that got published: \n " + message.toString());
         debugger.publishPlannedFootsteps(message);
      }
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      // Checks for how much time of the current step we have been planning for, stop planning if the step is close to completing
      boolean stopPlanningAndCompleteCurrentStep = stopWatch.getTime() > timeInSwingToStopPlanningAndWaitTillNextAttempt;
      return stopPlanningAndCompleteCurrentStep || (continuousPlanner.isPlanAvailable() && continuousHikingParameters.getStepPublisherEnabled());
   }

   public SideDependentList<FramePose3D> getGoalPosesBasedOnPlanningMode()
   {
      String message = commandMessage.get().toString();
      continuousHikingLogger.appendString("Continuous Hiking Command Being Used: \n" + message);

      SideDependentList<FramePose3D> goalPoses;

      if (!walkToGoalWayPointPoses.isEmpty())
      {
         // Allow for more planning time with this one, just plan for one-step length
         continuousHikingParameters.setPlanningWithoutReferenceTimeout(1.0);

         // Set the goalPoses here so that we return a good value regardless of what happens next
         goalPoses = walkToGoalWayPointPoses.get(0);

         // Update the current robot location
         Vector3DBasics robotLocationVector = referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame().getTranslation();
         robotLocation.set(robotLocationVector);
         double distanceToGoalPose = ContinuousPlannerTools.getDistanceFromRobotToGoalPoseOnXYPlane(robotLocation, goalPoses);

         if (distanceToGoalPose < continuousHikingParameters.getNextWaypointDistanceMargin())
         {
            walkToGoalWayPointPoses.remove(0);

            // If we have reached the goal, we can stop walking
            if (walkToGoalWayPointPoses.isEmpty())
            {
               commandMessage.get().setEnableContinuousHiking(false);
            }
         }
      }
      else if (commandMessage.get().getUseJoystickController())
      {
         if (commandMessage.get().getWalkBackwards())
         {
            goalPoses = ContinuousPlannerTools.setStraightBackwardGoalPoses(continuousPlanner.getWalkingStartMidPose(),
                                                                            continuousPlanner.getStartStancePose(),
                                                                            (float) continuousHikingParameters.getGoalPoseBackwardDistance(),
                                                                            (float) continuousHikingParameters.getGoalPoseUpDistance(),
                                                                            X_RANDOM_MARGIN,
                                                                            NOMINAL_STANCE_WIDTH);
         }
         // Here we assume the joystick isn't being turned at all, so we give a direction of straight forward
         // The number here it to account for drift in the controller where that value is never actually zero.
         // Cause the Joystick is drifting constantly
         else if (Math.abs(commandMessage.get().getLateralValue()) < 0.05)
         {
            previousLateralValue = 0;
            goalPoses = ContinuousPlannerTools.setStraightForwardGoalPoses(continuousPlanner.getWalkingStartMidPose(),
                                                                           continuousPlanner.getStartStancePose(),
                                                                           (float) continuousHikingParameters.getGoalPoseForwardDistance(),
                                                                           (float) continuousHikingParameters.getGoalPoseUpDistance(),
                                                                           X_RANDOM_MARGIN,
                                                                           NOMINAL_STANCE_WIDTH);
         }
         else
         {
            // Apply an alpha filter to the joystick value so we can't jump around so quickly.
            // Helps with the controller to perform better
            double lateralValue = commandMessage.get().getLateralValue();
            double filteredLateralValue = lateralValue * JOYSTICK_ALPHA + previousLateralValue *(1 - JOYSTICK_ALPHA);

            goalPoses = ContinuousPlannerTools.setGoalPoseBasedOnLateralJoystickValue(referenceFrames.getPelvisZUpFrame(),
                                                                                      referenceFrames.getMidFeetZUpFrame(),
                                                                                      filteredLateralValue,
                                                                                      (float) continuousHikingParameters.getGoalPoseForwardDistance(),
                                                                                      (float) continuousHikingParameters.getGoalPoseUpDistance(),
                                                                                      NOMINAL_STANCE_WIDTH);

            previousLateralValue = filteredLateralValue;

            // We update this pose because when we start walking straight forward again, it's from the point where we are currently at
            // And not the point from which we were at before we started turning
            FramePose3D stanceMidPose = new FramePose3D();
            stanceMidPose.interpolate(continuousPlanner.getStartStancePose().get(RobotSide.LEFT),
                                      continuousPlanner.getStartStancePose().get(RobotSide.RIGHT),
                                      0.5);
            continuousPlanner.setWalkingStartMidPose(stanceMidPose);
         }
      }
      else
      {
         if (commandMessage.get().getSideStep())
         {
            double sidewaysDistance = commandMessage.get().getLeftDirection() ?
                  continuousHikingParameters.getGoalPoseSidewaysDistance() :
                  -continuousHikingParameters.getGoalPoseSidewaysDistance();

            goalPoses = ContinuousPlannerTools.setSideStepGoalPoses(continuousPlanner.getWalkingStartMidPose(),
                                                                    continuousPlanner.getStartStancePose(),
                                                                    (float) sidewaysDistance,
                                                                    (float) continuousHikingParameters.getGoalPoseUpDistance(),
                                                                    X_RANDOM_MARGIN,
                                                                    NOMINAL_STANCE_WIDTH);
         }
         else if (commandMessage.get().getWalkBackwards())
         {
            goalPoses = ContinuousPlannerTools.setStraightBackwardGoalPoses(continuousPlanner.getWalkingStartMidPose(),
                                                                            continuousPlanner.getStartStancePose(),
                                                                            (float) continuousHikingParameters.getGoalPoseBackwardDistance(),
                                                                            (float) continuousHikingParameters.getGoalPoseUpDistance(),
                                                                            X_RANDOM_MARGIN,
                                                                            NOMINAL_STANCE_WIDTH);
         }
         else
         {
            goalPoses = ContinuousPlannerTools.setStraightForwardGoalPoses(continuousPlanner.getWalkingStartMidPose(),
                                                                           continuousPlanner.getStartStancePose(),
                                                                           (float) continuousHikingParameters.getGoalPoseForwardDistance(),
                                                                           (float) continuousHikingParameters.getGoalPoseUpDistance(),
                                                                           X_RANDOM_MARGIN,
                                                                           NOMINAL_STANCE_WIDTH);
         }
      }

      return goalPoses;
   }

   public void addWayPointPoseToList(PoseListMessage poseListMessage)
   {
      List<Pose3D> poses = MessageTools.unpackPoseListMessage(poseListMessage);
      FramePose3D leftFootPose = new FramePose3D();
      FramePose3D rightFootPose = new FramePose3D();

      leftFootPose.set(poses.get(0));
      rightFootPose.set(poses.get(1));

      SideDependentList<FramePose3D> latestWayPoint = new SideDependentList<>();
      latestWayPoint.put(RobotSide.LEFT, leftFootPose);
      latestWayPoint.put(RobotSide.RIGHT, rightFootPose);

      LogTools.info("Added waypoint for WALK_TO_GOAL");
      walkToGoalWayPointPoses.add(latestWayPoint);
      continuousPlanner.setStartStancePose(referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame(),
                                           referenceFrames.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());
      debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartStancePose(), latestWayPoint);
   }

   /**
    * This allows the {@link ReadyToPlanState#walkToGoalWayPointPoses} to be cleared.
    * This empties the list so the user can place a fresh goal that Continuous Hiking will use.
    */
   public void clearWayPointList()
   {
      LogTools.info("Clearing waypoint list for WALK_TO_GOAL");
      walkToGoalWayPointPoses.clear();
   }
}
