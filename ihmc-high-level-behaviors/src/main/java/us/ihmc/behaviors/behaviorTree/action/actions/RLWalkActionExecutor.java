package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.AbortWalkingMessage;
import controller_msgs.ControllerWaypointGoalListMessage;
import controller_msgs.ControllerWaypointGoalMessage;
import controller_msgs.ControllerWaypointStatusMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.CSGROS2CommunicationHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.tools.NonWallTimer;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Sends the goal to the RL walking goal reacher, then finishes when the controller reports the goal as reached.
 * The goal is taken from the parent frame when the action starts, so a goal in a robot frame does not move while walking.
 */
public class RLWalkActionExecutor extends ActionNodeExecutor<RLWalkActionState, RLWalkActionDefinition>
{
   /** The step generator drops waypoint goals until it has heard this heartbeat, so the goal waits this long after it starts */
   private static final double HEARTBEAT_LEAD_TIME = 1.0;
   private static final double GOAL_MATCH_EPSILON = 1.0e-3;
   /** Fail when walking takes longer than this many times the nominal duration plus the margin */
   private static final double TIMEOUT_SCALE = 3.0;
   private static final double TIMEOUT_MARGIN = 10.0;
   /** The controller default speed limits, used to estimate how long walking takes when the action uses them */
   private static final double CONTROLLER_MAX_FORWARD_SPEED = 0.5;
   private static final double CONTROLLER_MAX_TURNING_SPEED = 0.5;

   private final ROS2Heartbeat stepGeneratorHeartbeat;
   private boolean heartbeatAlive = false;
   private final NonWallTimer heartbeatAliveTimer = new NonWallTimer();
   private final ROS2Subscription<ControllerWaypointStatusMessage> waypointStatusSubscription;
   /** Every status is queued, since the goal reacher can publish several in the same tick */
   private final ConcurrentLinkedQueue<ControllerWaypointStatusMessage> receivedWaypointStatuses = new ConcurrentLinkedQueue<>();
   private final NonWallTimer executionTimer = new NonWallTimer();
   private final FramePose3D goalPose = new FramePose3D();
   private final FramePose3D midFeetPose = new FramePose3D();
   private boolean goalSent = false;

   public RLWalkActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new RLWalkActionState(id, rootNode.getState()), rootNode);

      stepGeneratorHeartbeat = new ROS2Heartbeat(ros2ControllerHelper.getROS2Node(), CSGROS2CommunicationHelper.CSG_HEARTBEAT_TOPIC);
      waypointStatusSubscription = ros2ControllerHelper.getROS2Node()
                                                       .createSubscriptionSampler(HumanoidControllerAPI.getTopic(ControllerWaypointStatusMessage.class,
                                                                                                                 ros2ControllerHelper.getRobotName()),
                                                                                  status -> receivedWaypointStatuses.add(new ControllerWaypointStatusMessage(status)));
   }

   @Override
   public void update()
   {
      super.update();

      double now = Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp());
      executionTimer.update(now);
      heartbeatAliveTimer.update(now);

      boolean canExecute = state.getGoalFrame().isChildOfWorld();
      state.setCanExecute(canExecute);
      if (!canExecute)
         cantExecuteMessage = "Parent frame %s is not in the scene.".formatted(definition.getParentFrameName());

      // Start the heartbeat before this action runs so the goal does not have to wait for it
      setHeartbeatAlive(state.getIsNextForExecution() || state.getIsExecuting());
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      receivedWaypointStatuses.clear();
      goalSent = false;
      executionTimer.reset();
      setHeartbeatAlive(true);

      if (!state.getGoalFrame().isChildOfWorld())
      {
         state.getLogger().error("Cannot execute. Frame is not a child of World frame.");
         state.setIsExecuting(false);
         state.setFailed(true);
         return;
      }

      goalPose.setToZero(state.getGoalFrame().getReferenceFrame());
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());

      midFeetPose.setFromReferenceFrame(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
      double distance = goalPose.getPosition().distanceXY(midFeetPose.getPosition());
      double turn = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(goalPose.getYaw(), midFeetPose.getYaw()));
      state.setNominalExecutionDuration(distance / getSpeedOrDefault(definition.getMaxForwardSpeed(), CONTROLLER_MAX_FORWARD_SPEED)
                                        + turn / getSpeedOrDefault(definition.getMaxTurningSpeed(), CONTROLLER_MAX_TURNING_SPEED));

      sendGoalWhenHeartbeatIsReady();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());

      midFeetPose.setFromReferenceFrame(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
      state.getCurrentPose().accessValue().set(midFeetPose);
      state.setPositionDistanceToGoalTolerance(definition.getPositionProximity());

      if (!goalSent)
         sendGoalWhenHeartbeatIsReady();

      ControllerWaypointStatusMessage status;
      while ((status = receivedWaypointStatuses.poll()) != null)
      {
         // A status that is not started means that goal was reached
         boolean isOurGoal = Math.abs(status.getGoalXPosition() - goalPose.getX()) < GOAL_MATCH_EPSILON
                             && Math.abs(status.getGoalYPosition() - goalPose.getY()) < GOAL_MATCH_EPSILON;
         if (goalSent && isOurGoal && !status.getIsStarted())
         {
            state.getLogger().info("Reached the RL walk goal");
            stopWalking();
            state.setIsExecuting(false);
            return;
         }
      }

      double timeout = TIMEOUT_SCALE * state.getNominalExecutionDuration() + TIMEOUT_MARGIN;
      if (!executionTimer.isRunning(timeout))
      {
         state.getLogger().error("Did not reach the RL walk goal within %.1f s".formatted(timeout));
         stopWalking();
         state.setIsExecuting(false);
         state.setFailed(true);
      }
   }

   private void sendGoalWhenHeartbeatIsReady()
   {
      if (!heartbeatAliveTimer.isExpired(HEARTBEAT_LEAD_TIME))
         return;

      ControllerWaypointGoalListMessage goalListMessage = new ControllerWaypointGoalListMessage();
      ControllerWaypointGoalMessage goalMessage = goalListMessage.getWaypoints().add();
      goalMessage.setXPosition(goalPose.getX());
      goalMessage.setYPosition(goalPose.getY());
      goalMessage.setYaw(goalPose.getYaw());
      goalMessage.setPositionProximity(definition.getPositionProximity());
      goalMessage.setKeepHeading(definition.getHeadingMode() == RLWalkHeadingMode.KEEP_HEADING);
      goalMessage.setTurnWalkTurn(definition.getHeadingMode() == RLWalkHeadingMode.TURN_WALK_TURN);
      goalMessage.setMaxForwardSpeed(definition.getMaxForwardSpeed());
      goalMessage.setMaxTurningSpeed(definition.getMaxTurningSpeed());

      state.getLogger().info("Sending RL walk goal x %.2f y %.2f yaw %.2f".formatted(goalPose.getX(), goalPose.getY(), goalPose.getYaw()));
      ros2ControllerHelper.publishToController(goalListMessage);
      goalSent = true;
   }

   /**
    * The goal list leaves the step generator set to walk, so it would step in place once the controller switches to
    * WALKING. Aborting clears that and the goal. This is sent while the heartbeat is still alive, or the step generator
    * drops it.
    */
   private void stopWalking()
   {
      ros2ControllerHelper.publishToController(new AbortWalkingMessage());
   }

   private void setHeartbeatAlive(boolean alive)
   {
      if (alive != heartbeatAlive)
      {
         stepGeneratorHeartbeat.setAlive(alive);
         heartbeatAlive = alive;
         if (alive)
            heartbeatAliveTimer.reset();
      }
   }

   private static double getSpeedOrDefault(double speed, double defaultSpeed)
   {
      return speed > 0.0 ? speed : defaultSpeed;
   }

   @Override
   public void destroy()
   {
      super.destroy();

      ros2ControllerHelper.getROS2Node().destroySubscription(waypointStatusSubscription);
      stepGeneratorHeartbeat.destroy();
   }
}
