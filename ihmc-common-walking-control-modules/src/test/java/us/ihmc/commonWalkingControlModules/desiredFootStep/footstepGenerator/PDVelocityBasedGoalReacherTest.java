package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import controller_msgs.ControllerWalkToGoalStatusMessage;
import controller_msgs.ControllerWaypointStatusMessage;
import controller_msgs.VelocityBasedWalkingInputMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWaypointGoalCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWaypointGoalListCommand;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class PDVelocityBasedGoalReacherTest
{
   @Test
   public void defaultTwoMetreReverseTurnsInPlaceToFaceTheGoal()
   {
      VelocityBasedWalkingInputMessage output = commandToward(-2.0, 0.0, false);
      assertTrue(Math.abs(output.getForwardVelocity()) < 1.0e-6 && Math.abs(output.getLateralVelocity()) < 1.0e-6,
                 "Default far reverse should turn in place before walking, got forward=" + output.getForwardVelocity()
                 + " lateral=" + output.getLateralVelocity());
      assertTrue(Math.abs(output.getTurnVelocity()) > 0.1, "Default far reverse should turn to face the goal, got turn=" + output.getTurnVelocity());
   }

   @Test
   public void everyModeReachesGoalsFacingAnotherWay()
   {
      double[][] goals = {{2.0, 0.0, 90.0}, {2.0, 0.0, 180.0}, {-2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {1.0, 1.0, -90.0}, {0.5, 0.0, 180.0}, {3.0, -2.0, 45.0}};
      for (HeadingMode mode : HeadingMode.values())
      {
         for (double[] goal : goals)
         {
            SimulatedWalk walk = simulateWalk(mode, new double[][] {goal});
            String description = mode + " to (" + goal[0] + ", " + goal[1] + ", " + goal[2] + " deg)";
            assertTrue(walk.reached, description + " did not reach the goal");
            assertTrue(Math.abs(walk.finalYawError) < 0.1 + 1.0e-3, description + " ended " + Math.toDegrees(walk.finalYawError) + " deg off the goal yaw");
            double straightLine = Math.hypot(goal[0], goal[1]);
            assertTrue(walk.pathLength < straightLine + 0.2, description + " walked " + walk.pathLength + " m for a " + straightLine + " m goal");
         }
      }
   }

   @Test
   public void keepHeadingWalksBackwardToAGoalBehind()
   {
      SimulatedWalk walk = simulateWalk(HeadingMode.KEEP_HEADING, new double[][] {{-2.0, 0.0, 0.0}});
      assertTrue(walk.reached);
      assertTrue(walk.maxYawDeviationFromStart < Math.toRadians(10.0), "Keep heading turned " + Math.toDegrees(walk.maxYawDeviationFromStart) + " deg");
   }

   @Test
   public void turnWalkTurnStaysOnTheStartToGoalLine()
   {
      SimulatedWalk walk = simulateWalk(HeadingMode.TURN_WALK_TURN, new double[][] {{3.0, -2.0, 45.0}});
      assertTrue(walk.reached);
      assertTrue(walk.maxDistanceFromLine < 0.05, "Turn-walk-turn strayed " + walk.maxDistanceFromLine + " m off the line");
   }

   @Test
   public void everyModeWalksASquareOfWaypoints()
   {
      for (HeadingMode mode : HeadingMode.values())
      {
         SimulatedWalk walk = simulateWalk(mode, new double[][] {{2.0, 0.0, 90.0}, {2.0, 2.0, 180.0}, {0.0, 2.0, -90.0}, {0.0, 0.0, 0.0}});
         assertTrue(walk.reached, mode + " did not finish the square");
         assertTrue(walk.pathLength < 8.2, mode + " walked " + walk.pathLength + " m around a 8 m square");
      }
   }

   @Test
   public void walkToGoalStatusReportsASingleGoalAsReached()
   {
      PoseReferenceFrame base = new PoseReferenceFrame("base", ReferenceFrame.getWorldFrame());
      StatusMessageOutputManager status = new StatusMessageOutputManager(List.of(ControllerWalkToGoalStatusMessage.class, ControllerWaypointStatusMessage.class));
      boolean[] reached = {false};
      status.attachStatusMessageListener(ControllerWalkToGoalStatusMessage.class, message -> reached[0] |= message.getIsReached());
      PDVelocityBasedGoalReacher reacher = new PDVelocityBasedGoalReacher(base, status, new YoRegistry("test"));

      FramePose2D goal = new FramePose2D(ReferenceFrame.getWorldFrame());
      goal.set(2.0, 1.0, 0.0);
      ControllerWaypointGoalCommand command = new ControllerWaypointGoalCommand();
      command.setGoalPose(goal);
      command.setPositionProximity(0.1);
      command.setOrientationProximity(0.1);
      command.setGoalOrientationMatters(false);
      reacher.consumeNewWaypoint(command);

      double dt = 0.02;
      double x = 0.0, y = 0.0, yaw = 0.0;
      for (double time = 0.0; time < 60.0 && !reached[0]; time += dt)
      {
         base.setPoseAndUpdate(new Pose3D(x, y, 0.0, yaw, 0.0, 0.0));
         reacher.update(time);
         VelocityBasedWalkingInputMessage output = reacher.getOutputMessage();
         if (output == null || !output.getWalk())
            continue;
         x += (output.getForwardVelocity() * Math.cos(yaw) - output.getLateralVelocity() * Math.sin(yaw)) * dt;
         y += (output.getForwardVelocity() * Math.sin(yaw) + output.getLateralVelocity() * Math.cos(yaw)) * dt;
         yaw += output.getTurnVelocity() * dt;
      }
      assertTrue(reached[0], "The walk-to-goal status never reported the goal as reached, robot ended at (" + x + ", " + y + ")");
      assertTrue(Math.hypot(2.0 - x, 1.0 - y) < 0.1 + 1.0e-3, "Reported reached at (" + x + ", " + y + ")");
   }

   private enum HeadingMode
   {
      FACE_GOAL, KEEP_HEADING, TURN_WALK_TURN
   }

   private static class SimulatedWalk
   {
      boolean reached;
      double finalYawError;
      double pathLength;
      double maxYawDeviationFromStart;
      double maxDistanceFromLine;
   }

   /**
    * Walks a robot that tracks the commanded velocities perfectly from the origin, facing +x, through the waypoints
    * (x, y, yaw in degrees) with the RDX waypoint settings.
    */
   private static SimulatedWalk simulateWalk(HeadingMode mode, double[][] waypoints)
   {
      PoseReferenceFrame base = new PoseReferenceFrame("base", ReferenceFrame.getWorldFrame());
      StatusMessageOutputManager status = new StatusMessageOutputManager(List.of(ControllerWalkToGoalStatusMessage.class, ControllerWaypointStatusMessage.class));
      SimulatedWalk walk = new SimulatedWalk();
      status.attachStatusMessageListener(ControllerWaypointStatusMessage.class, message ->
      {
         if (!message.getIsStarted() && message.getWaypointsRemaining() == 0)
            walk.reached = true;
      });
      PDVelocityBasedGoalReacher reacher = new PDVelocityBasedGoalReacher(base, status, new YoRegistry("test"));

      ControllerWaypointGoalListCommand list = new ControllerWaypointGoalListCommand();
      for (double[] waypoint : waypoints)
      {
         ControllerWaypointGoalCommand command = new ControllerWaypointGoalCommand();
         FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame());
         goalPose.set(waypoint[0], waypoint[1], Math.toRadians(waypoint[2]));
         command.setGoalPose(goalPose);
         command.setPositionProximity(0.1);
         command.setOrientationProximity(0.1);
         command.setGoalOrientationMatters(true);
         command.setKeepHeading(mode == HeadingMode.KEEP_HEADING);
         command.setTurnWalkTurn(mode == HeadingMode.TURN_WALK_TURN);
         list.addWaypoint(command);
      }
      reacher.consumeNewWaypointList(list);

      double[] goal = waypoints[waypoints.length - 1];
      double lineLength = Math.hypot(goal[0], goal[1]);
      double dt = 0.02;
      double x = 0.0, y = 0.0, yaw = 0.0;
      for (double time = 0.0; time < 120.0 && !walk.reached; time += dt)
      {
         base.setPoseAndUpdate(new Pose3D(x, y, 0.0, yaw, 0.0, 0.0));
         reacher.update(time);
         VelocityBasedWalkingInputMessage output = reacher.getOutputMessage();
         if (output == null || !output.getWalk())
            continue;
         double dx = (output.getForwardVelocity() * Math.cos(yaw) - output.getLateralVelocity() * Math.sin(yaw)) * dt;
         double dy = (output.getForwardVelocity() * Math.sin(yaw) + output.getLateralVelocity() * Math.cos(yaw)) * dt;
         x += dx;
         y += dy;
         yaw += output.getTurnVelocity() * dt;
         walk.pathLength += Math.hypot(dx, dy);
         if (Math.hypot(goal[0] - x, goal[1] - y) > 0.2)
            walk.maxYawDeviationFromStart = Math.max(walk.maxYawDeviationFromStart, Math.abs(AngleTools.trimAngleMinusPiToPi(yaw)));
         walk.maxDistanceFromLine = Math.max(walk.maxDistanceFromLine, Math.abs(x * goal[1] - y * goal[0]) / lineLength);
      }
      walk.finalYawError = AngleTools.computeAngleDifferenceMinusPiToPi(yaw, Math.toRadians(goal[2]));
      return walk;
   }

   @Test
   public void keepHeadingTwoMetreReverseCommandsBackwardNotAForwardArc()
   {
      VelocityBasedWalkingInputMessage output = commandToward(-2.0, 0.0, true);
      assertTrue(output.getForwardVelocity() < 0.0,
                 "A keep-heading 2 m reverse must command backward speed, got forward=" + output.getForwardVelocity()
                 + " turn=" + output.getTurnVelocity());
      assertTrue(Math.abs(output.getTurnVelocity()) < 0.05,
                 "A keep-heading reverse with matching yaw must not turn to face the goal, got turn=" + output.getTurnVelocity());
   }

   @Test
   public void aHalfMetreReverseStillCommandsBackward()
   {
      VelocityBasedWalkingInputMessage output = commandToward(-0.5, 0.0, false);
      assertTrue(output.getForwardVelocity() < 0.0,
                 "A 0.5 m reverse must command backward speed, got " + output.getForwardVelocity());
   }

   @Test
   public void aTwoMetreForwardGoalStillCommandsForward()
   {
      VelocityBasedWalkingInputMessage output = commandToward(2.0, 0.0, false);
      assertTrue(output.getForwardVelocity() > 0.0,
                 "A 2 m forward goal must command forward speed, got " + output.getForwardVelocity());
   }

   @Test
   public void keepHeadingTwoMetreSidestepCommandsLateralNotAForwardArc()
   {
      VelocityBasedWalkingInputMessage output = commandToward(0.0, 2.0, true);
      assertTrue(Math.abs(output.getLateralVelocity()) > Math.abs(output.getForwardVelocity()),
                 "A keep-heading 2 m sidestep must command more lateral than forward speed, got forward="
                 + output.getForwardVelocity() + " lateral=" + output.getLateralVelocity());
      assertTrue(Math.abs(output.getTurnVelocity()) < 0.05,
                 "A keep-heading sidestep with matching yaw must not turn to face the goal, got turn=" + output.getTurnVelocity());
   }

   private static VelocityBasedWalkingInputMessage commandToward(double x, double y, boolean keepHeading)
   {
      PoseReferenceFrame base = new PoseReferenceFrame("base", ReferenceFrame.getWorldFrame());
      StatusMessageOutputManager status = new StatusMessageOutputManager(List.of(ControllerWalkToGoalStatusMessage.class, ControllerWaypointStatusMessage.class));
      PDVelocityBasedGoalReacher reacher = new PDVelocityBasedGoalReacher(base, status, new YoRegistry("test"));

      FramePose2D goal = new FramePose2D(ReferenceFrame.getWorldFrame());
      goal.set(x, y, 0.0);
      ControllerWaypointGoalCommand command = new ControllerWaypointGoalCommand();
      command.setGoalPose(goal);
      command.setPositionProximity(0.25);
      command.setOrientationProximity(Math.toRadians(30.0));
      command.setGoalOrientationMatters(false);
      command.setKeepHeading(keepHeading);
      reacher.consumeNewWaypoint(command);

      reacher.update(0.0);
      reacher.update(0.1);
      reacher.update(0.2);

      VelocityBasedWalkingInputMessage output = reacher.getOutputMessage();
      assertNotNull(output, "Reacher produced no walking command");
      return output;
   }
}
