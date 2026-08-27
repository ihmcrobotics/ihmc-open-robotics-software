package us.ihmc.avatar.walkToGoal;

import controller_msgs.AbortWalkingMessage;
import controller_msgs.ControllerWalkToGoalStatusMessage;
import controller_msgs.ControllerWaypointGoalListMessage;
import controller_msgs.ControllerWaypointGoalMessage;
import controller_msgs.VelocityBasedWalkingInputMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.CSGROS2CommunicationHelper;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.tools.Timer;

/** Plants a walk-to-goal pose, or takes the body back with a streamed velocity. */
public class WalkToGoalClient
{
   private static final Vector2DReadOnly ZERO_LINEAR_VELOCITY = new Vector2D();
   private static final double GOAL_RESEND_PERIOD = 0.2;
   private static final double GOAL_RESEND_TIMEOUT = 2.0;
   private static final double GOAL_MATCH_EPSILON = 1.0e-3;

   private final ROS2ControllerHelper controllerHelper;
   private final ControllerWaypointGoalListMessage waypointList = new ControllerWaypointGoalListMessage();
   private final AbortWalkingMessage abortMessage = new AbortWalkingMessage();
   private final VelocityBasedWalkingInputMessage velocityMessage = new VelocityBasedWalkingInputMessage();
   private final FramePose2D worldGoal = new FramePose2D();
   /** StepGeneratorCommandInputManager discards goals unless this heartbeat is alive. */
   private final ROS2Heartbeat stepGeneratorHeartbeat;
   private final ROS2Input<ControllerWalkToGoalStatusMessage> goalStatus;
   private final Pose2D requestedGoal = new Pose2D();
   private final Timer resendTimer = new Timer();
   private final Throttler resendThrottler = new Throttler().setPeriod(GOAL_RESEND_PERIOD);
   private boolean waitingForGoalToBeAccepted = false;
   private boolean goalActive = false;
   private double positionProximity = 0.20;
   private double orientationProximity = Math.toRadians(30.0);
   private long sequenceId = 1L;

   public WalkToGoalClient(ROS2ControllerHelper controllerHelper)
   {
      this.controllerHelper = controllerHelper;
      stepGeneratorHeartbeat = new ROS2Heartbeat(controllerHelper.getROS2Node(), CSGROS2CommunicationHelper.CSG_HEARTBEAT_TOPIC);
      stepGeneratorHeartbeat.setAlive(true);
      goalStatus = controllerHelper.subscribeToController(ControllerWalkToGoalStatusMessage.class);
   }

   /** Replace the current goal with this pose, expressed in any frame. */
   public void goTo(FramePose2DReadOnly goal)
   {
      worldGoal.setIncludingFrame(goal);
      worldGoal.changeFrame(ReferenceFrame.getWorldFrame());
      goTo((Pose2DReadOnly) worldGoal);
   }

   /** Replace the current goal with this world pose. Uses a one-waypoint list so it does not append. */
   public void goTo(Pose2DReadOnly goal)
   {
      goTo(goal, 0.20, Math.toRadians(30.0));
   }

   public void goTo(Pose2DReadOnly goal, double positionProximity, double orientationProximity)
   {
      requestedGoal.set(goal);
      this.positionProximity = positionProximity;
      this.orientationProximity = orientationProximity;
      goalActive = true;
      waitingForGoalToBeAccepted = true;
      resendTimer.reset();
      publishGoal();
   }

   /**
    * The step generator drops incoming commands whenever its heartbeat monitor lapses, which a single
    * publish cannot detect, so resend the goal until the controller reports it as the current goal.
    */
   public void update()
   {
      if (!waitingForGoalToBeAccepted)
         return;

      if (isRequestedGoalReported() || !resendTimer.isRunning(GOAL_RESEND_TIMEOUT))
         waitingForGoalToBeAccepted = false;
      else if (resendThrottler.run())
         publishGoal();
   }

   private boolean isRequestedGoalReported()
   {
      if (!goalStatus.hasReceivedFirstMessage())
         return false;

      ControllerWalkToGoalStatusMessage status = goalStatus.getLatest();
      return Math.abs(status.getCurrentGoalXPosition() - requestedGoal.getX()) < GOAL_MATCH_EPSILON
             && Math.abs(status.getCurrentGoalYPosition() - requestedGoal.getY()) < GOAL_MATCH_EPSILON;
   }

   private void publishGoal()
   {
      waypointList.setSequenceId(sequenceId++);
      waypointList.getWaypoints().clear();
      ControllerWaypointGoalMessage waypoint = waypointList.getWaypoints().add();
      waypoint.setXPosition(requestedGoal.getX());
      waypoint.setYPosition(requestedGoal.getY());
      waypoint.setYaw(requestedGoal.getYaw());
      waypoint.setHoldPosition(true);
      waypoint.setPositionProximity(positionProximity);
      waypoint.setOrientationProximity(orientationProximity);
      controllerHelper.publish(HumanoidControllerAPI.getTopic(ControllerWaypointGoalListMessage.class, controllerHelper.getRobotName()), waypointList);
   }

   public void commandVelocity(Vector2DReadOnly bodyLinearVelocity, double turnVelocity, boolean walk)
   {
      velocityMessage.setWalk(walk);
      velocityMessage.setAreVelocitiesNormalized(false);
      velocityMessage.setForwardVelocity(bodyLinearVelocity.getX());
      velocityMessage.setLateralVelocity(bodyLinearVelocity.getY());
      velocityMessage.setTurnVelocity(turnVelocity);
      controllerHelper.publishToController(velocityMessage);
   }

   public void stop()
   {
      abortGoal();
      commandVelocity(ZERO_LINEAR_VELOCITY, 0.0, false);
   }

   public void abortGoal()
   {
      waitingForGoalToBeAccepted = false;
      if (!goalActive)
         return;
      goalActive = false;
      controllerHelper.publish(HumanoidControllerAPI.getTopic(AbortWalkingMessage.class, controllerHelper.getRobotName()), abortMessage);
   }

   public void destroy()
   {
      stepGeneratorHeartbeat.destroy();
      goalStatus.destroy();
   }
}
