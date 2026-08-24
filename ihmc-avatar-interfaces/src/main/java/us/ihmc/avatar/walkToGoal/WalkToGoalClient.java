package us.ihmc.avatar.walkToGoal;

import controller_msgs.AbortWalkingMessage;
import controller_msgs.ControllerWaypointGoalListMessage;
import controller_msgs.ControllerWaypointGoalMessage;
import controller_msgs.ControllerWaypointListStatusMessage;
import controller_msgs.ControllerWaypointStatusMessage;
import controller_msgs.VelocityBasedWalkingInputMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.CSGROS2CommunicationHelper;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.tools.Timer;

import java.util.List;

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
   private final FixedFramePose2DBasics worldGoal = new FramePose2D();
   /** StepGeneratorCommandInputManager discards goals unless this heartbeat is alive. */
   private final ROS2Heartbeat stepGeneratorHeartbeat;
   private final ROS2Input<ControllerWaypointListStatusMessage> waypointListStatus;
   private final Timer resendTimer = new Timer();
   private final Throttler resendThrottler = new Throttler().setPeriod(GOAL_RESEND_PERIOD);
   private boolean waitingForGoalToBeAccepted = false;
   private long sequenceId = 1L;

   public WalkToGoalClient(ROS2ControllerHelper controllerHelper)
   {
      this.controllerHelper = controllerHelper;
      stepGeneratorHeartbeat = new ROS2Heartbeat(controllerHelper.getROS2Node(), CSGROS2CommunicationHelper.CSG_HEARTBEAT_TOPIC);
      stepGeneratorHeartbeat.setAlive(true);
      waypointListStatus = controllerHelper.subscribeToController(ControllerWaypointListStatusMessage.class);
   }

   /** Replace the current goal with this pose, expressed in any frame. */
   public void goTo(FramePose2DReadOnly goal)
   {
      worldGoal.setMatchingFrame(goal);
      goTo((Pose2DReadOnly) worldGoal);
   }

   /** Replace the current goal with this world pose. Uses a one-waypoint list so it does not append. */
   public void goTo(Pose2DReadOnly... goals)
   {
      goTo(0.20, Math.toRadians(30.0), goals);
   }

   /** Replace the current goal with this world pose. Uses a one-waypoint list so it does not append. */
   public void goTo(double positionProximity, double orientationProximity, Pose2DReadOnly... goals)
   {
      waitingForGoalToBeAccepted = true;
      resendTimer.reset();
      setWaypointListFromGoals(positionProximity, orientationProximity, goals);
   }

   /** Replace the current goal with this world pose. Uses a one-waypoint list so it does not append. */
   public void goTo(double positionProximity, double orientationProximity, Pose3DReadOnly... goals)
   {
      waitingForGoalToBeAccepted = true;
      resendTimer.reset();
      setWaypointListFromGoals(positionProximity, orientationProximity, goals);
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
      {
         waitingForGoalToBeAccepted = false;
         waypointList.getWaypoints().clear();
      }
      else if (resendThrottler.run())
      {
         publishGoals();
      }
   }

   private boolean isRequestedGoalReported()
   {
      if (!waypointListStatus.hasReceivedFirstMessage())
         return false;

      double firstWaypointX = waypointList.getWaypoints().get(0).getXPosition();
      double firstWaypointY = waypointList.getWaypoints().get(0).getYPosition();
      IDLObjectSequence<ControllerWaypointStatusMessage> queuedWaypoints = waypointListStatus.getLatest().getWaypoints();
      for (int i = 0; i < queuedWaypoints.size(); i++)
      {
         ControllerWaypointStatusMessage queuedWaypoint = queuedWaypoints.get(i);
         if (EuclidCoreTools.epsilonEquals(queuedWaypoint.getGoalXPosition(), firstWaypointX, GOAL_MATCH_EPSILON)
             && EuclidCoreTools.epsilonEquals(queuedWaypoint.getGoalYPosition(), firstWaypointY, GOAL_MATCH_EPSILON))
         {
            return true;
         }
      }
      return false;
   }

   private void setWaypointListFromGoals(double positionProximity, double orientationProximity, Pose2DReadOnly... goals)
   {
      waypointList.setSequenceId(sequenceId++);
      waypointList.getWaypoints().clear();
      for (Pose2DReadOnly goal : goals)
      {
         addWaypointToList(positionProximity, orientationProximity, goal.getX(), goal.getY(), goal.getYaw());
      }
   }

   private void setWaypointListFromGoals(double positionProximity, double orientationProximity, Pose3DReadOnly... goals)
   {
      waypointList.setSequenceId(sequenceId++);
      waypointList.getWaypoints().clear();
      for (Pose3DReadOnly goal : goals)
      {
         addWaypointToList(positionProximity, orientationProximity, goal.getX(), goal.getY(), goal.getYaw());
      }
   }

   private void addWaypointToList(double positionProximity, double orientationProximity, double xPosition, double yPosition, double yaw)
   {
      ControllerWaypointGoalMessage waypoint = waypointList.getWaypoints().add();
      waypoint.setXPosition(xPosition);
      waypoint.setYPosition(yPosition);
      waypoint.setYaw(yaw);
      waypoint.setPositionProximity(positionProximity);
      waypoint.setOrientationProximity(orientationProximity);
   }

   private void publishGoals()
   {
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
      controllerHelper.publish(HumanoidControllerAPI.getTopic(AbortWalkingMessage.class, controllerHelper.getRobotName()), abortMessage);
   }

   public void destroy()
   {
      stepGeneratorHeartbeat.destroy();
      waypointListStatus.destroy();
   }
}
