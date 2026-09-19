package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import controller_msgs.ControllerWalkToGoalStatusMessage;
import controller_msgs.ControllerWaypointStatusMessage;
import controller_msgs.VelocityBasedWalkingInputMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWaypointGoalCommand;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class PDVelocityBasedGoalReacherTest
{
   @Test
   public void defaultTwoMetreReverseFacesTheGoalAndWalksForward()
   {
      VelocityBasedWalkingInputMessage output = commandToward(-2.0, 0.0, false);
      assertTrue(output.getForwardVelocity() > 0.0,
                 "Default far reverse should face the goal and walk forward, got forward=" + output.getForwardVelocity()
                 + " turn=" + output.getTurnVelocity());
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
